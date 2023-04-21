import {defineStore} from "pinia";

type Color = {}
type Vector2 = { x: number, y: number };
type DrawingMethod = {};
type Visualization = {};

type DrawingMsg = {
    retainForTicks: number,
    robotId: number | null,
    color: Color,
    method: DrawingMethod,
    points: Vector2[],
}

type VisualizationStoreStateT = {
    drawings: DrawingMsg[],
}
export const useVisualizationStore = defineStore('visualizationStore', {
    state: (): VisualizationStoreStateT => { return {
        drawings: []
    }},
    actions: {
        addDrawing(drawing: DrawingMsg) {
            this.drawings.push(drawing);
        },
        tick() {
            this.drawings = this.drawings.filter(drawing => drawing.retainForTicks > 0);
            this.drawings.forEach(drawing => drawing.retainForTicks--);
        }
    },
});

export {}