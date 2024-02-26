<script setup lang='ts'>
// Internal (non-reactive) variables
import { inject, onBeforeUnmount, watch } from 'vue'
import { appSymbol } from './utils'
import { useSTPDataStore } from '../../stores/data-stores/stp-data-store'
import { useUIStore } from '../../stores/ui-store'
import { useVisualizationStore } from '../../stores/data-stores/visualization-store'
import { proto } from '../../../generated/proto'
import Category = proto.Drawing.Category
import { NoUndefinedField } from '../../../utils'
import IDrawing = proto.IDrawing
import { ShapeDrawing, ShapeMap } from './field-objects'

let visuals = new ShapeMap<string, ShapeDrawing>()

const
  app = inject(appSymbol)!,
  stpData = useSTPDataStore(),
  visualizationStore = useVisualizationStore(),
  uiStore = useUIStore()

const onPixiTick = () => {
  const buffer = visualizationStore.popAllDrawings()
  buffer.forEach((data) => {
    if (data.category == Category.PATH_PLANNING && !uiStore.showPathPlanning(data.forRobotId))
      return
    if (data.category == Category.DEBUG && !uiStore.showDebug(data.forRobotId)) return
    if (data.category == Category.MARGINS && !uiStore.showMargins(data.forRobotId)) return

    const shape = new ShapeDrawing({
      data: data as NoUndefinedField<IDrawing>,
      currentTick: stpData.currentTick
    })

    app.value.layers.drawings.addChild(shape)
    visuals.set(data.label!, shape) // Drawings map automatically calls destroy on the old shape
  })

  visuals.removeExpiredShapes(stpData.currentTick)
}

const cleanUp = () => {
  visuals.forEach((shape) => shape.destroy({ children: true }))
  visuals.clear()
  app.value.ticker.remove(onPixiTick)
}

watch(app, (_, __, onCleanup) => {
  app.value.ticker.add(onPixiTick)
  onCleanup(cleanUp)
}, { immediate: true })
onBeforeUnmount(cleanUp)
</script>

<template>
  <div hidden>
    <!--  Visualization Component (Nothing to render, rendering is done using pixi not Vue) -->
  </div>
</template>
