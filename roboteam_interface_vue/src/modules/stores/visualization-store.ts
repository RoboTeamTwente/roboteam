import {defineStore} from "pinia";
import {proto} from "../../generated/proto";
import {ref, shallowRef} from "vue";
import IDrawingBuffer = proto.MessageEnvelope.IDrawingBuffer;
import IMetricBuffer = proto.MessageEnvelope.IMetricBuffer;


export const useVisualizationStore = defineStore('useVisStore', () => {
    const drawingsBuffer = shallowRef<Readonly<proto.IDrawing[]>>([]);
    const metrics = ref<Map<string, proto.IMetric>>(new Map());

    const pushMetrics = (msg: IMetricBuffer) => {
        msg.buffer?.forEach(metric => {
            if (metric.decimal != null) {
                let decimal = {
                    ...metrics.value.get(metric.label!)?.decimal,
                    ...metric.decimal
                };

                if ((decimal.maxRecorded ?? decimal.value!) <= decimal.value!) {
                    decimal.maxRecorded = decimal.value!;
                }

                if ((decimal.minRecorded ?? decimal.value!) >= decimal.value!) {
                    decimal.minRecorded = decimal.value!;
                }

                metrics.value.set(metric.label!, {
                    label: metric.label,
                    decimal: decimal
                });
            } else {
                metrics.value.set(metric.label!, metric);
            }
        });
    }

    const pushDrawings = (drawingBuffer: IDrawingBuffer) => {
        if (drawingsBuffer.value.length + (drawingBuffer.buffer?.length ?? 0)  > 250) {
            console.warn("Too many drawings, removing oldest");
            drawingsBuffer.value = drawingsBuffer.value.slice(drawingBuffer.buffer?.length ?? 0);
        }

        drawingsBuffer.value = [...drawingsBuffer.value, ...drawingBuffer.buffer!];
    };

    const popAllDrawings = () => {
        const drawings = drawingsBuffer.value;
        drawingsBuffer.value = [];
        return drawings;
    }

    return {
        metrics,
        pushMetrics,
        popAllDrawings,
        pushDrawings
    }
});
