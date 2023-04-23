import {defineStore} from "pinia";
import {proto} from "../../generated/proto";
import {shallowRef} from "vue";


export const useVisualizationStore = defineStore('useVisStore', () => {
    const drawingsBuffer = shallowRef<Readonly<proto.IDrawing[]>>([]);
    const push = (drawing: proto.IDrawing) => {
        if (drawingsBuffer.value.length > 250) {
            console.warn("Too many drawings, removing oldest");
            drawingsBuffer.value = drawingsBuffer.value.slice(1);
        }

        drawingsBuffer.value = [...drawingsBuffer.value, drawing];
    }

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
