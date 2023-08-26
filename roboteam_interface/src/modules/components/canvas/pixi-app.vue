<script setup lang='ts'>
import { computed, onBeforeUnmount, onMounted, provide, ref, ShallowRef, shallowRef, watch } from 'vue'
import { appSymbol, stageSymbol, useMoveCamera, usePointerLocation } from './utils'
import { useEventListener } from '@vueuse/core'
import { Container } from '@pixi/display'
import { DisplayObject } from 'pixi.js'
import { Colors, CustomPixiApplication } from './field-objects'

const props = defineProps<{
  length: number
  width: number
}>()

const
  canvas = ref<HTMLCanvasElement | null>(null),
  app = shallowRef<CustomPixiApplication | null>(null)

const stage = computed(() => app.value?.stage ?? null)

// Since children are only rendered after the app is initialized,
// we don't need to worry about the app or stage being null
provide(appSymbol, app as ShallowRef<CustomPixiApplication>)
provide(stageSymbol, stage as ShallowRef<Container<DisplayObject>>)

useMoveCamera(canvas, app)
usePointerLocation(stage, app)

onMounted(() => {
  app.value = new CustomPixiApplication({
    width: (props.length! / 10) * 1.15,
    height: (props.width! / 10) * 1.15,
    backgroundColor: Colors.backgroundColor,
    view: canvas.value!
  })

  // Setup click events
  app.value.stage.eventMode = 'static'
  app.value.stage.hitArea = app.value.screen
})

onBeforeUnmount(() => {
  app.value?.stage.removeAllListeners()
  app.value?.stage.destroy(false)
})

watch([() => props.length, () => props.width],
  () => app.value?.renderer.resize(props.width, props.length)
)

useEventListener(canvas, 'contextmenu', (event) => {
  event.preventDefault()
  return false
})

</script>
<template>
  <canvas
    class='min-m-6 m-auto min-h-0 min-w-0 max-h-full max-w-full w-auto h-auto rounded-xl'
    ref='canvas'
  />
  <template v-if='app !== null'>
    <slot></slot>
  </template>
</template>
