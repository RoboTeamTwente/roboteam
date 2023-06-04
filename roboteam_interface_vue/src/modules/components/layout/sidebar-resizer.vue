<script setup lang="ts">
import { computed, ref } from 'vue'
import { usePointerSwipe } from '@vueuse/core'

const props = defineProps<{
  direction: 'x' | 'y'
}>()

const emit = defineEmits<{
  (e: 'drag', traveledDist: { x: number; y: number }): void
}>()

const sliderElem = ref(null)
let lastPos = { x: 0, y: 0 }
const { distanceX, distanceY } = usePointerSwipe(sliderElem, {
  onSwipeStart(e: PointerEvent) {
    e.preventDefault()
    lastPos = { x: 0, y: 0 }
  },
  onSwipeEnd(e: PointerEvent) {
    e.preventDefault()
    lastPos = { x: 0, y: 0 }
  },
  onSwipe(e: PointerEvent) {
    e.preventDefault()
    const delta = {
      x: distanceX.value - lastPos.x,
      y: distanceY.value - lastPos.y
    }
    lastPos = { x: distanceX.value, y: distanceY.value }

    emit('drag', delta)
  }
})

const classes = computed(() => {
  return {
    'cursor-ew-resize': props.direction === 'x',
    'cursor-ns-resize': props.direction === 'y'
  }
})
</script>

<template>
  <div ref="sliderElem" class="bg-base-300" :class="[$attrs.class, classes]"></div>
</template>

<style scoped></style>
