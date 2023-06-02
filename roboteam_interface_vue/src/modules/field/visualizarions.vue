<script setup lang="ts">

import {Application, Container} from "pixi.js";
import {CustomPixiApplication, ShapeDrawing, ShapeMap} from "./field-objects";
import {onMounted, onUnmounted, watch} from "vue";
import {proto} from "../../generated/proto";
import {useUIStore} from "../stores/ui-store";
import {useVisualizationStore} from "../stores/data-stores/visualization-store";
import Category = proto.Drawing.Category;
import {useSTPDataStore} from "../stores/data-stores/stp-data-store";

// Internal (non-reactive) variables
let visuals = new ShapeMap<string, ShapeDrawing>(),
    layer: null | Container = null;

// Reactive values
const props = defineProps<{
        app: CustomPixiApplication,
    }>(),
    visualizationStore = useVisualizationStore(),
    uiStore = useUIStore(),
    stpData = useSTPDataStore();

// Methods
const
    init = () => {
        layer = new Container();
        props.app.drawingsContainer.addChild(layer);
        props.app.ticker.add(onPixiTick);
    },
    cleanUp = () => {
        console.log("Cleaning up visualization");
        layer?.destroy({children: true});
        props.app.ticker.remove(onPixiTick);
    },
    onPixiTick = () => {
        const buffer = visualizationStore.popAllDrawings();
        buffer.forEach(props => {
            if (props.category == Category.PATH_PLANNING && !uiStore.showPathPlanning(props.forRobotId)) return;
            if (props.category == Category.DEBUG && !uiStore.showDebug(props.forRobotId)) return;

            const shape = new ShapeDrawing({data: props, currentTick: stpData.currentTick});
            layer?.addChild(shape);
            visuals.set(props.label!, shape); // Drawings map automatically calls destroy on the old shape
        });

        visuals.removeExpiredShapes(stpData.currentTick);

        visuals.set("test", new ShapeDrawing(
            {
                data: {
                    size: 190,
                    category: Category.PATH_PLANNING,
                    forRobotId: -1,
                    color: proto.Drawing.Color.RED,
                    points: [{
                        x: 0,
                        y: 0
                    }],
                    label: "test",
                    method: proto.Drawing.Method.DOTS,
                    retainForTicks: 100,
                },
                currentTick: stpData.currentTick
            }
        ));
        layer?.addChild(visuals.get("test")!);
    };

watch([() => props.app], () => {
    cleanUp();
    init();
}, {immediate: true});
onUnmounted(cleanUp);

</script>

<template>
  <!--  Visualization Component (Nothing to render, rendering is done using pixi not Vue) -->
</template>