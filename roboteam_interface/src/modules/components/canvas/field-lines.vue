<script setup lang='ts'>

// Reactive values
import { proto } from '../../../generated/proto'
import ISSL_GeometryFieldSize = proto.ISSL_GeometryFieldSize
import { DeepReadonly, inject, onBeforeUnmount, shallowRef, ShallowRef, watch } from 'vue'
import { appSymbol } from './utils'
import { FieldDrawing } from './field-objects'

const props = defineProps<{
  isYellow: boolean
  fieldGeometry: DeepReadonly<ISSL_GeometryFieldSize>
}>()

const
  app = inject(appSymbol)!,
  field = shallowRef<FieldDrawing | null>(null)

watch(
  [app, props],
  (_, __, onCleanup) => {

    field.value = new FieldDrawing({
      fieldGeometry: props.fieldGeometry,
      isYellow: props.isYellow
    })

    app.value.layers.fieldLines.addChild(field.value)
    onCleanup(() => field.value?.destroy({ children: true }))
  }, { immediate: true })

onBeforeUnmount(() => {
  field.value?.destroy({ children: true })
})

</script>
<template>
  <!-- Field Component -->
</template>
