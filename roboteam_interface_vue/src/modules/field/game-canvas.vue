<script setup lang="ts">
import { FederatedPointerEvent } from 'pixi.js'
import { onMounted, onUnmounted, ref, shallowRef, ShallowRef, watch } from 'vue'
import { Colors, CustomPixiApplication } from './field-objects'

import { useVisionDataStore } from '../stores/data-stores/vision-data-store'
import { useAIDataStore } from '../stores/data-stores/ai-data-store'
import Robots from './robots-graphics.vue'
import Ball from './ball-graphics.vue'
import Field from './field-graphics.vue'
import Visualizations from './pointer-location.vue'
import PointerLocation from './visualizarions-graphics.vue'
import { useAiController } from '../composables/ai-controller'

const canvas = ref<HTMLCanvasElement | null>(null),
  appRef: ShallowRef<null | CustomPixiApplication> = shallowRef(null),
  visionData = useVisionDataStore(),
  aiData = useAIDataStore(),
  aiController = useAiController()

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

    app.stage.addEventListener('pointerdown', onStageClick)
    appRef.value = app
  },
  onStageClick = (e: FederatedPointerEvent) => {
    if (e.button != 0 || !e.shiftKey) {
      return
    } // Only allow left click + shift
    const pos = e.getLocalPosition(appRef.value!.drawingsContainer)
    aiController.setBallPos(pos.x / 100, -pos.y / 100)
    e.preventDefault()
  },
  cleanUp = () => {
    console.log('cleaning up game canvas')
    appRef.value?.stage.removeEventListener('pointerdown', onStageClick)
    appRef.value?.destroy(false)

    appRef.value = null
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
    class="min-m-6 m-auto min-h-0 min-w-0 max-h-full max-w-full w-auto h-auto rounded-xl"
    ref="canvas"
  />
  <template v-if="appRef !== null">
    <pointer-location :app="appRef" />
    <field
      :app="appRef"
      :field-geometry="visionData.latestField"
      :is-yellow="aiData.state?.gameSettings?.isYellow!"
    >
      <!-- Order matters, since the order is equivalent to the order in which layers are added -->
      <robots :app="appRef" />
      <ball :app="appRef" />
      <visualizations :app="appRef" />
    </field>
  </template>
</template>
