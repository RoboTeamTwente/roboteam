<script setup lang='ts'>
import { FederatedPointerEvent } from 'pixi.js'
import { onMounted, onUnmounted, ref, shallowRef, ShallowRef, watch } from 'vue'
import { Colors, CustomPixiApplication } from './field-objects'

import { useVisionDataStore } from '../stores/data-stores/vision-data-store'
import { useAIDataStore } from '../stores/data-stores/ai-data-store'
import Robots from './robots-graphics.vue'
import Ball from './ball-graphics.vue'
import Field from './field-graphics.vue'
import PointerLocation from './pointer-location.vue'
import Visualizations from './visualizations-graphics.vue'
import { useAiController } from '../composables/ai-controller'
import { useUIStore } from '../stores/ui-store'
import { proto } from '../../generated/proto'
import { useThrottleFn } from '@vueuse/core'

const canvas = ref<HTMLCanvasElement | null>(null),
  appRef: ShallowRef<null | CustomPixiApplication> = shallowRef(null),
  visionData = useVisionDataStore(),
  aiData = useAIDataStore(),
  aiController = useAiController(),
  uiStore = useUIStore()

const isLeftButtonPressed = ref(false)

const init = (length: number, width: number) => {
    const app = new CustomPixiApplication({
      width: (length! / 10) * 1.15,
      height: (width! / 10) * 1.15,
      backgroundColor: Colors.backgroundColor,
      view: canvas.value!
    })

    // Setup click events
    app.stage.eventMode = 'static'
    app.stage.hitArea = app.screen

    app.stage.addEventListener('pointerup', onPointerUp)
    app.stage.addEventListener('pointerdown', onPointerDown)
    app.stage.addEventListener('pointermove', onPointerMove)
    appRef.value = app
  },
  onPointerDown = (e: FederatedPointerEvent) => {
    isLeftButtonPressed.value = e.button === 0
    onPointerMove(e)
  },
  onPointerUp = () => isLeftButtonPressed.value = false,
  onPointerMove = useThrottleFn((e: FederatedPointerEvent) => {
    if (!isLeftButtonPressed.value) return

    // shift + left click
    const pos = (() => {
      const pixiPos = e.getLocalPosition(appRef.value!.drawingsContainer)
      return { x: pixiPos.x / 100, y: -pixiPos.y / 100 }
    })()

    if (e.shiftKey) {
      aiController.sendSimulatorCommand({
        control: { teleportBall: { ...pos } }
      })
    }


    if (e.altKey) {
      const robots = [...uiStore.selectedRobots].map(id => ({
        id: { id: id, 'team': aiData.state!.gameSettings!.isYellow ? proto.Team.YELLOW : proto.Team.BLUE },
        orientation: 0,
        ...pos
      }))

      aiController.sendSimulatorCommand({
        control: {
          teleportRobot: robots
        }
      })
    }


    e.preventDefault()
    return
  }, 50),
  cleanUp = () => {
    console.log('cleaning up game canvas')
    appRef.value?.stage.removeEventListener('pointerup', onPointerUp)
    appRef.value?.stage.removeEventListener('pointerdown', onPointerMove)
    appRef.value?.stage.removeEventListener('pointermove', onPointerMove)
    appRef.value?.destroy(false)

    appRef.value = null
    isLeftButtonPressed.value = false
  }

watch(
  () => visionData.latestField,
  () => {
    console.log('field changed')
    cleanUp()
    if (visionData.latestField !== null) {
      init(visionData.latestField!.fieldLength!, visionData.latestField!.fieldWidth!)
    }
  },
  { immediate: true }
)

onMounted(() => {
  console.log('mounting game canvas')
  cleanUp()
  if (visionData.latestField !== null) {
    init(visionData.latestField!.fieldLength!, visionData.latestField!.fieldWidth!)
  }
})

onUnmounted(() => {
  console.log('unmounting game canvas')
  cleanUp()
})
</script>
<template>
  <canvas
    class='min-m-6 m-auto min-h-0 min-w-0 max-h-full max-w-full w-auto h-auto rounded-xl'
    ref='canvas'
  />
  <template v-if='appRef !== null'>
    <field
      :app='appRef'
      :field-geometry='visionData.latestField'
      :is-yellow='aiData.state?.gameSettings?.isYellow!'
    >
      <!-- Order matters, since the order is equivalent to the order in which layers are added -->
      <robots :app='appRef' />
      <ball :app='appRef' />
      <visualizations :app='appRef' />
      <pointer-location :app='appRef' />
    </field>
  </template>
</template>
