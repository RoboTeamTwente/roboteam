<script setup lang="ts">
import { FederatedPointerEvent, Text } from 'pixi.js'
import { CustomPixiApplication } from './field-objects'
import { onUnmounted, shallowRef, watch } from 'vue'

// Reactive values
const props = defineProps<{
    app: CustomPixiApplication
  }>(),
  cursorRef = shallowRef<Text | null>(null)

// Methods
const init = () => {
    console.log('Setting up pointer location')

    // Init cursor position text
    const cursor = new Text('', { fontSize: 16, fill: 'white' })
    cursor.x = props.app.screen.width * 0.025
    cursor.y = props.app.screen.height * 0.025
    props.app.stage.addChild(cursor)

    cursorRef.value = cursor
    props.app.stage.addEventListener('pointerleave', onPointerLeave)
    props.app.stage.addEventListener('pointermove', onPointerMove)
  },
  onPointerLeave = () => {
    cursorRef.value!.text = ''
  },
  onPointerMove = (e: FederatedPointerEvent) => {
    // TODO make [0,0] the center of the field
    const pos = e.getLocalPosition(props.app.stage)
    const pos_x = (pos.x - props.app.stage.width / 2) / 100
    const pos_y = (pos.y - props.app.stage.height / 2) / 100
    console.log(props.app.stage.width)
    cursorRef.value!.text = `(TODO fix) [${pos_x.toFixed(2)}x, ${(pos_y).toFixed(2)}y]`
  },
  cleanUp = () => {
    console.log('Cleaning up pointer location')
    props.app.stage.removeAllListeners('pointermove')
    props.app.stage.removeAllListeners('pointerleave')
    props.app.stage.removeEventListener('pointerleave', onPointerLeave)
    props.app.stage.removeEventListener('pointermove', onPointerMove)

    cursorRef.value?.destroy()
    cursorRef.value = null
  }

watch(
  () => props.app,
  () => {
    cleanUp()
    init()
  },
  { immediate: true }
)
onUnmounted(cleanUp)
// onBeforeUnmount(cleanUp);
</script>

<template>
  <div hidden>
    <!-- Pointer Component (Nothing to render, rendering is done using pixi not Vue) -->
  </div>
</template>
