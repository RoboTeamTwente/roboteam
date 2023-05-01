<script setup lang="ts">
import {Application, Container, Text,} from "pixi.js";
import {DeepReadonly, onMounted, onUnmounted, ref, toRaw, watch} from "vue";
import {BallDrawing, Colors, FieldDrawing, RobotDrawing, ShapeDrawing, useFieldObjectStorage} from "./field-objects";

import {useGameSettingsStore} from "../stores/game-settings-store";
import {proto} from "../../generated/proto";
import {useUIStore} from "../stores/ui-store";
import {useAIStore} from "../stores/ai-store";
import {useVisualizationStore} from "../stores/visualization-store";
import IWorldRobot = proto.IWorldRobot;
import Category = proto.Drawing.Category;

const canvas = ref<HTMLCanvasElement | null>(null);

const gameStore = useGameSettingsStore();
const uiStore = useUIStore();
const aiStore = useAIStore();
const visualizationStore = useVisualizationStore();

let app: null | Application = null;
let {clearRobotsDrawings, layers, drawings} = useFieldObjectStorage();

//Each AI tick, remove drawings that are too old
watch(() => aiStore.stpData.currentTick, (currentTick) => drawings.shapes.removeExpiredShapes(currentTick));

// re-init the canvas on field change, since the field size could have change
watch(() => aiStore.visionData.latestField, () => initPixiApp());

// Clear drawings when team changes
watch(() => gameStore.team, () => {
    clearRobotsDrawings();

    // When the team changes, we also have to redraw the field to update the goal colors
    drawField(aiStore.visionData.latestField);
});

watch(() => uiStore.scaling.ball, () => scale());
watch(() => uiStore.scaling.robots, () => scale());
const scale = () => {
    drawings.ball.scale.set(uiStore.scaling.ball);
    drawings.blueRobots.forEach(robot => robot.scale.set(uiStore.scaling.robots));
    drawings.yellowRobots.forEach(robot => robot.scale.set(uiStore.scaling.robots));
};


const updateRobotDrawing = (drawingsMap: Map<number, RobotDrawing>, robot: IWorldRobot, team: 'yellow' | 'blue') => {
    const robotId = robot.id ?? -1;
    if (!drawingsMap.has(robotId)) {
        const drawing = new RobotDrawing({
            team: team,
            text: robotId.toString(),
            onClick: team == gameStore.team
                ? () => uiStore.toggleRobotSelection(robotId)
                : undefined,
        });

        drawing.scale.set(uiStore.scaling.robots);
        layers.movingObjects.addChild(drawing);
        drawingsMap.set(robotId, drawing);
    }
    const drawing = drawingsMap.get(robotId)!;
    drawing.toggleSelection(toRaw(uiStore.isaRobotSelected(robotId)) && team == gameStore.team);
    drawing.moveOnField(gameStore.fieldOrientation.x * robot.pos!.x!, gameStore.fieldOrientation.y * robot.pos!.y!, gameStore.fieldOrientation.angle + -(robot.angle ?? 0));
    drawing.setVelocity(team == gameStore.team && uiStore.showVelocities(robotId), gameStore.fieldOrientation.x * robot.vel!.x!, gameStore.fieldOrientation.y * robot.vel!.y!);
}

const drawField = (field: DeepReadonly<proto.ISSL_GeometryFieldSize> | null) => {
    console.debug("Redraw field");

    // Remove old field drawing
    layers.fieldGeometry.removeChildren(0).forEach((child) => child.destroy());

    if (field === null) return;
    layers.fieldGeometry.addChild(new FieldDrawing({
        fieldGeometry: field,
        fieldColors: toRaw(gameStore.goalColor),
    }));
}

const initPixiApp = () => {
    console.debug("Init Pixi app");
    app = new Application({
        width: aiStore.visionData.latestField?.fieldLength! / 10 * 1.15,
        height: aiStore.visionData.latestField?.fieldWidth! / 10 * 1.15,
        backgroundColor: Colors.backgroundColor,
        view: canvas.value!
    });

    // Init field geometry drawings
    drawField(aiStore.visionData.latestField!);

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
    const world = aiStore.visionData.latestWorld;
    if (world == null) {
        return;
    }

    // Update robot drawings
    world.yellow!.forEach(robot => updateRobotDrawing(drawings.yellowRobots, robot, 'yellow'));
    world.blue!.forEach(robot => updateRobotDrawing(drawings.blueRobots, robot, 'blue'));

    // Update ball drawing
    drawings.ball.moveOnField(gameStore.fieldOrientation.x * world!.ball!.pos!.x!, gameStore.fieldOrientation.y * world.ball!.pos!.y!);

    // Draw the latest shapes received from the AI
    const buffer = visualizationStore.popAllDrawings();
    buffer.forEach(props => {
        if (props.category == Category.PATH_PLANNING && !uiStore.showPathPlanning(props.forRobotId)) return;
        if (props.category == Category.DEBUG && !uiStore.showDebug(props.forRobotId)) return;

        const shape = new ShapeDrawing({data: props, currentTick: aiStore.stpData.currentTick});
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
