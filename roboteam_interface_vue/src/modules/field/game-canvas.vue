<script setup lang="ts">
import {Application, Container, Text,} from "pixi.js";
import {DeepReadonly, onMounted, onUnmounted, ref, toRaw, watch} from "vue";
import {BallDrawing, Colors, FieldDrawing, RobotDrawing, ShapeDrawing, useFieldObjectStorage} from "./field-objects";

import {proto} from "../../generated/proto";
import {useUIStore} from "../stores/ui-store";
import {useGameControllerStore} from "../stores/ai-store";
import {useVisualizationStore} from "../stores/dataStores/visualization-store";
import IWorldRobot = proto.IWorldRobot;
import Category = proto.Drawing.Category;
import {useSTPDataStore} from "../stores/dataStores/stp-data-store";
import {useVisionDataStore} from "../stores/dataStores/vision-data-store";
import {watchDeep} from "@vueuse/core";
import {useAIDataStore} from "../stores/dataStores/ai-data-store";

const canvas = ref<HTMLCanvasElement | null>(null);

const uiStore = useUIStore();

const stpData = useSTPDataStore();
const visionData = useVisionDataStore();
const visualizationStore = useVisualizationStore();
const gameController = useGameControllerStore();
const aiData = useAIDataStore();

let app: null | Application = null;
let {clearRobotsDrawings, layers, drawings} = useFieldObjectStorage();

//Each AI tick, remove drawings that are too old
watch(() => stpData.currentTick, (currentTick) => drawings.shapes.removeExpiredShapes(currentTick));

// re-init the canvas on field change, since the field size could have change
watch(() => visionData.latestField, () => initPixiApp());

// Clear drawings when team changes
watch(() => aiData.state!.gameSettings?.isYellow, () => {
    clearRobotsDrawings();
    drawField(visionData.latestField); // When the team changes, we also have to redraw the field to update the goal colors
});

// When the scaling setting changes, update the scaling of the drawings
watchDeep(() => uiStore.scaling, () => {
    drawings.ball.scale.set(uiStore.scaling.ball);
    drawings.blueRobots.forEach(robot => robot.scale.set(uiStore.scaling.robots));
    drawings.yellowRobots.forEach(robot => robot.scale.set(uiStore.scaling.robots));
});


const updateRobotDrawing = (drawingsMap: Map<number, RobotDrawing>, robot: IWorldRobot, isYellow: boolean) => {
    const robotId = robot.id ?? -1;
    const isUs = isYellow == aiData.state!.gameSettings?.isYellow // Only allow clicking on robots of your own team
    const fieldOrientation = aiData.fieldOrientation;

    if (!drawingsMap.has(robotId)) {
        const drawing = new RobotDrawing({
            isYellow: isYellow,
            text: robotId.toString(),
            onClick: isUs
                ? () => uiStore.toggleRobotSelection(robotId)
                : undefined,
        });

        drawing.scale.set(uiStore.scaling.robots);
        layers.movingObjects.addChild(drawing);
        drawingsMap.set(robotId, drawing);
    }

    drawingsMap.get(robotId)!.update(isUs && uiStore.isaRobotSelected(robotId), isUs, fieldOrientation, robot);
}

const drawField = (field: DeepReadonly<proto.ISSL_GeometryFieldSize> | null) => {
    console.log("Drawing field");

    // Remove old field drawing
    layers.fieldGeometry.removeChildren(0).forEach((child) => child.destroy());

    if (field === null) return;
    layers.fieldGeometry.addChild(new FieldDrawing({
        fieldGeometry: field,
        fieldColors: toRaw(aiData.goalColor),
    }));
}

const initPixiApp = () => {
    app = new Application({
        width: visionData.latestField?.fieldLength! / 10 * 1.15,
        height: visionData.latestField?.fieldWidth! / 10 * 1.15,
        backgroundColor: Colors.backgroundColor,
        view: canvas.value!
    });

    // Init field geometry drawings
    drawField(visionData.latestField!);

    // Init cursor position text
    const cursor = new Text("", {fontSize: 16, fill: 'white'});
    cursor.x = app.screen.width * 0.025;
    cursor.y = app.screen.height * 0.025;

    // Init ball drawing
    drawings.ball = new BallDrawing();
    drawings.ball.scale.set(uiStore.scaling.ball);

    layers.movingObjects.addChild(drawings.ball!);

    // this puts the (0, 0) coordinates to the center of the stage
    const container = new Container();
    container.x = app.screen.width / 2;
    container.y = app.screen.height / 2;

    // Add layers to the stage (ORDER MATTERS)
    app.stage.addChild(container);
    container.addChild(layers.fieldGeometry, layers.movingObjects, layers.shapeLayer);
    app.stage.addChild(cursor)

    // Setup mouse position text
    app.stage.eventMode = 'static';
    app.stage.hitArea = app.screen;
    app.stage.addEventListener('pointerleave', () => {
        cursor.text = "";
    });
    app.stage.addEventListener('pointermove', (e) => {
        const pos = e.getLocalPosition(container);
        cursor.text = `[${((pos.x) / 100).toFixed(2)}x, ${((pos.y) / 100).toFixed(2)}y]`
    });

    app.ticker.add(onPixiTick);
}

const onPixiTick = () => {
    const world = visionData.latestWorld;
    if (world == null) {
        return;
    }

    // Update robot drawings
    world.yellow!.forEach(robot => updateRobotDrawing(drawings.yellowRobots, robot, true));
    world.blue!.forEach(robot => updateRobotDrawing(drawings.blueRobots, robot, false));

    // Update ball drawing
    drawings.ball.moveOnField(aiData.fieldOrientation.x * world!.ball!.pos!.x!, aiData.fieldOrientation.y * world.ball!.pos!.y!);

    // Draw the latest shapes received from the AI
    const buffer = visualizationStore.popAllDrawings();
    buffer.forEach(props => {
        if (props.category == Category.PATH_PLANNING && !uiStore.showPathPlanning(props.forRobotId)) return;
        if (props.category == Category.DEBUG && !uiStore.showDebug(props.forRobotId)) return;

        const shape = new ShapeDrawing({data: props, currentTick: stpData.currentTick});
        layers.shapeLayer.addChild(shape);
        drawings.shapes.set(props.label!, shape); // Drawings map automatically calls destroy on the old shape
    });
}

onMounted(initPixiApp);
onUnmounted(async () => app?.destroy());

</script>
<template>
    <canvas class="min-m-6 m-auto min-h-0 min-w-0 max-h-full max-w-full w-auto h-auto rounded-xl" ref="canvas"></canvas>
</template>
