<script setup lang='ts'>
import { computed, onMounted, onUnmounted, ref, shallowRef, ShallowRef, watch } from 'vue'
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
import { useFieldZoom, useMoveBall, useMoveRobots } from './utils'

const canvas = ref<HTMLCanvasElement | null>(null),
  appRef: ShallowRef<null | CustomPixiApplication> = shallowRef(null),
  visionData = useVisionDataStore(),
  aiData = useAIDataStore(),
  aiController = useAiController(),
  uiStore = useUIStore()

const stage = computed(() => appRef.value?.stage)


useFieldZoom(canvas, appRef)
useMoveBall(canvas, appRef, stage)
useMoveRobots(canvas, appRef, stage)

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
    appRef.value = app
  },
  cleanUp = () => {
    console.log('cleaning up game canvas')
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

  canvas.value?.addEventListener('contextmenu', (event) => {
    event.preventDefault()
    return false
  })
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
