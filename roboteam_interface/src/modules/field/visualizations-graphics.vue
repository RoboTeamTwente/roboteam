<script setup lang="ts">
import { Container } from 'pixi.js'
import { CustomPixiApplication, ShapeDrawing, ShapeMap } from './field-objects'
import { onUnmounted, watch } from 'vue'
import { proto } from '../../generated/proto'
import { useUIStore } from '../stores/ui-store'
import { useVisualizationStore } from '../stores/data-stores/visualization-store'
import Category = proto.Drawing.Category
import { useSTPDataStore } from '../stores/data-stores/stp-data-store'
import { NoUndefinedField } from '../../utils'
import IDrawing = proto.IDrawing

// Internal (non-reactive) variables
let visuals = new ShapeMap<string, ShapeDrawing>(),
  layer: null | Container = null

// Reactive values
const props = defineProps<{
    app: CustomPixiApplication
  }>(),
  visualizationStore = useVisualizationStore(),
  uiStore = useUIStore(),
  stpData = useSTPDataStore()

// Methods
const init = () => {
    layer = new Container()
    props.app.drawingsContainer.addChild(layer)
    props.app.ticker.add(onPixiTick)
  },
  cleanUp = () => {
    console.log('Cleaning up visualization')
    layer?.destroy({ children: true })
    props.app.ticker.remove(onPixiTick)
  },
  onPixiTick = () => {
    const buffer = visualizationStore.popAllDrawings()
    buffer.forEach((data) => {
      if (data.category == Category.PATH_PLANNING && !uiStore.showPathPlanning(data.forRobotId))
        return
      if (data.category == Category.DEBUG && !uiStore.showDebug(data.forRobotId)) return

      const shape = new ShapeDrawing({
        data: data as NoUndefinedField<IDrawing>,
        currentTick: stpData.currentTick
      })
      layer?.addChild(shape)
      visuals.set(data.label!, shape) // Drawings map automatically calls destroy on the old shape
    })

    visuals.removeExpiredShapes(stpData.currentTick)
  }

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
    <!--  Visualization Component (Nothing to render, rendering is done using pixi not Vue) -->
  </div>
</template>
