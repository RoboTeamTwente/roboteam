import { defineStore } from 'pinia'
import { proto } from '../../../generated/proto'
import { readonly, ref, shallowRef } from 'vue'
import IMetric = proto.IMetric
import IDrawing = proto.IDrawing

export const useVisualizationStore = defineStore('useVisStore', () => {
  const drawingsBuffer = shallowRef<Readonly<proto.IDrawing[]>>([])
  const metrics = ref<Map<string, proto.IMetric>>(new Map())

  const pushMetrics = (msg: IMetric[]) => {
    msg.forEach((metric) => {
      if (metric.decimal != null) {
        const decimal = {
          ...metrics.value.get(metric.label!)?.decimal,
          ...metric.decimal
        }

        if ((decimal.maxRecorded ?? decimal.value!) <= decimal.value!) {
          decimal.maxRecorded = decimal.value!
        }

        if ((decimal.minRecorded ?? decimal.value!) >= decimal.value!) {
          decimal.minRecorded = decimal.value!
        }

        metrics.value.set(metric.label!, {
          label: metric.label,
          decimal: decimal
        })
      } else {
        metrics.value.set(metric.label!, metric)
      }
    })
  }

  const pushDrawings = (drawingBuffer: IDrawing[]) => {
    if (drawingsBuffer.value.length + (drawingBuffer.length ?? 0) > 250) {
      console.warn('Too many drawings, removing oldest')
      drawingsBuffer.value = drawingsBuffer.value.slice(drawingBuffer.length ?? 0)
    }

    drawingsBuffer.value = [...drawingsBuffer.value, ...drawingBuffer]
  }

  const popAllDrawings = () => {
    const drawings = drawingsBuffer.value
    drawingsBuffer.value = []
    return drawings
  }

  return {
    metrics: readonly(metrics),
    pushMetrics,
    popAllDrawings,
    pushDrawings
  }
})
