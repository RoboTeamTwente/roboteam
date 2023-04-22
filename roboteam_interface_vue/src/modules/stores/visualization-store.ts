import {defineStore} from "pinia";
import {proto} from "../../generated/proto";
import IDrawing = proto.IDrawing;
import {toRaw} from "vue";


type VisualizationStoreStateT = {
    drawings: IDrawing[],
}
export const useVisualizationStore = defineStore('visualizationStore', {
    state: (): VisualizationStoreStateT => { return {
        drawings: []
    }},
    actions: {
        addDrawing(drawing: IDrawing) {
            if (this.drawings.length > 250) {
                console.warn("Too many drawings, removing oldest");
                this.drawings.shift();
            }

            this.drawings.push(drawing as IDrawing);
        },
        popAll(){
            return toRaw(this.drawings.splice(0, this.drawings.length));
        },
        tick() {
            this.drawings = this.drawings.filter(drawing => drawing.retainForTicks! > 0);
            this.drawings.forEach(drawing => drawing.retainForTicks!--);
        }
    },
});

export {}