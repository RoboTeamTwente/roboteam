<script setup lang="ts">
import { BallDrawing, CustomPixiApplication } from './field-objects'
import { onUnmounted, shallowRef, ShallowRef, watch } from 'vue'
import { useVisionDataStore } from '../stores/data-stores/vision-data-store'
import { useUIStore } from '../stores/ui-store'
import { useAIDataStore } from '../stores/data-stores/ai-data-store'

// Internal (non-reactive) variables

// Reactive values
const props = defineProps<{
    app: CustomPixiApplication
  }>(),
  ballRef: ShallowRef<BallDrawing | null> = shallowRef(null),
  visionData = useVisionDataStore(),
  uiStore = useUIStore(),
  aiData = useAIDataStore()

// Methods
const init = () => {
    const ball = new BallDrawing()
    props.app.drawingsContainer.addChild(ball)
    props.app.ticker.add(onPixiTick)
    ballRef.value = ball
  },
  cleanUp = () => {
    console.log('Cleaning up ball drawing')
    ballRef.value?.destroy({ children: true })
    props.app.ticker.remove(onPixiTick)
  },
  onPixiTick = () => {
    const world = visionData.latestWorld
    ballRef.value?.moveOnField(
      aiData.fieldOrientation.x * world!.ball!.pos!.x!,
      aiData.fieldOrientation.y * world!.ball!.pos!.y!
    )
  }

watch([() => uiStore.scaling.ball, () => ballRef.value], () =>
  ballRef.value?.scale.set(uiStore.scaling.ball)
)

watch(
  [() => props.app],
  () => {
    cleanUp()
    init()
  },
  { immediate: true }
)

onUnmounted(cleanUp)
</script>

<template>
  <div hidden>
    <!--  Ball Component (Nothing to render, rendering is done using pixi not Vue) -->
  </div>
</template>
