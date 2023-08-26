<script setup lang='ts'>

import { inject, onBeforeUnmount, shallowRef, ShallowRef, watch } from 'vue'
import { useVisionDataStore } from '../../stores/data-stores/vision-data-store'
import { OUT_OF_CANVAS_COORDINATES } from '../../../utils'
import { useAIDataStore } from '../../stores/data-stores/ai-data-store'
import { useUIStore } from '../../stores/ui-store'
import { appSymbol, stageSymbol, useMoveBall } from './utils'
import { BallDrawing } from './field-objects'


const
  app = inject(appSymbol)!,
  stage = inject(stageSymbol)!,
  ball: ShallowRef<BallDrawing | null> = shallowRef(null),
  visionData = useVisionDataStore(),
  aiData = useAIDataStore(),
  uiStore = useUIStore()

useMoveBall(app, stage)

const onPixiTick = () => {
  const world = visionData.latestWorld
  if (world?.ball == null) {
    ball.value?.moveOnField(OUT_OF_CANVAS_COORDINATES.x, OUT_OF_CANVAS_COORDINATES.y) // Move the ball out of the canvas to hide it
    return
  }

  ball.value?.moveOnField(
    aiData.fieldOrientation.x * world.ball.pos!.x!,
    aiData.fieldOrientation.y * world.ball.pos!.y!
  )
}

watch(
  () => app,
  (app, _, onCleanup) => {
    ball.value = new BallDrawing()
    ball.value.scale.set(uiStore.scaling.ball)

    app.value?.layers.ball.addChild(ball.value)
    app.value?.ticker.add(onPixiTick)

    onCleanup(() => {
      ball.value?.destroy({ children: true })
      app.value?.ticker.remove(onPixiTick)
    })
  }, { immediate: true }
)

onBeforeUnmount(() => {
  ball.value?.destroy({ children: true })
  app.value?.ticker.remove(onPixiTick)
})

watch(() => uiStore.scaling.ball, () => ball.value?.scale.set(uiStore.scaling.ball))

</script>

<template>
  <div hidden>
    <!--  Ball Component (Nothing to render, rendering is done using pixi not Vue) -->
  </div>
</template>
