import {defineStore} from "pinia";
import {proto} from "../../generated/proto";
import {shallowRef} from "vue";


export const useVisualizationStore = defineStore('useVisStore', () => {
    const drawingsBuffer = shallowRef<Readonly<proto.IDrawing[]>>([]);

    const push = (drawingBuffer: proto.IDrawingBuffer) => {
        if (drawingsBuffer.value.length + (drawingBuffer.buffer?.length ?? 0)  > 250) {
            console.warn("Too many drawings, removing oldest");
            drawingsBuffer.value = drawingsBuffer.value.slice(drawingBuffer.buffer?.length ?? 0);
        }

        drawingsBuffer.value = [...drawingsBuffer.value, ...drawingBuffer.buffer!];
    };

    const popAll = () => {
        const drawings = drawingsBuffer.value;
        drawingsBuffer.value = [];
        return drawings;
    }

    return {
        popAll,
        push,
    }
});
