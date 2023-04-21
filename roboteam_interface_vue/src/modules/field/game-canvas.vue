<script setup lang="ts">
import {Application, Container, Graphics, Text} from "pixi.js";
import {computed, markRaw, onBeforeUpdate, onMounted, onUnmounted, onUpdated, ref, watch} from "vue";
import {BallDrawing, CanvasSettings, Colors, RobotDrawing, Size} from "./FieldObjects";
import {useWorldStateStore} from "../stores/world-store";
import {useGameSettingsStore} from "../stores/game-settings-store";
import {proto} from "../../generated/proto";
import IWorldRobot = proto.IWorldRobot;
import ISSL_GeometryData = proto.ISSL_GeometryData;
import IVector2 = proto.IVector2;

const props = defineProps<{
  size: Size;
}>();

let center: IVector2 = {x: 0, y: 0};

const mousePos = ref<IVector2>({x: 0, y: 0});
const canvas = ref<HTMLCanvasElement | null>(null);
const worldStore = useWorldStateStore();
const gameStore = useGameSettingsStore();

// TODO: Clean them on prop update
let app: null | Application = null;
let fieldGeometryLayer = new Container();
let movingObjectsLayer = new Container();
let cursor = new Text("", {fontSize: 16, fill: 'white'});
cursor.anchor.set(0, 1.5);

let drawings = markRaw({
  blueRobots: new Map<number, RobotDrawing>(),
  yellowRobots: new Map<number, RobotDrawing>(),
  ball: {} as BallDrawing, // Ball is initialized in onMounted, so it's not a problem
});

const colors = computed(() => {
  return {
    leftGoal: gameStore.$state.team === "yellow" ? Colors.yellow : Colors.blue,
    rightGoal: gameStore.$state.team === "yellow" ? Colors.blue : Colors.yellow,
  }
});

const flipSide = computed(() => {
  return {
    x: gameStore.$state.side === "left" ? 1 : -1,
    y: gameStore.$state.side === "left" ? -1 : 1,
  };
});

let filedBuffer: Graphics[] = [];

const drawFiled = (geometryData: ISSL_GeometryData) => {
  filedBuffer.forEach((graphics) => {graphics.destroy();});
  filedBuffer = [];

  geometryData.field.fieldLines?.forEach((line) => {
    // console.log(line.name)
    const graphics = new Graphics();
    let color = 'white';
    let thickness = 1;
    if (line.name === "LeftGoalDepthLine") {
      color = colors.value.leftGoal
      thickness = 2;
    }

    if (line.name === "RightGoalDepthLine") {
      color = colors.value.rightGoal
      thickness = 2;
    }

    graphics.lineStyle(thickness, color, 1);
    graphics.moveTo(center.x  + line.p1!.x!  / 1000 * 100, center.y + line.p1!.y!  / 1000 * 100);
    graphics.lineTo(center.x  + line.p2!.x!  / 1000 * 100, center.y + line.p2!.y!  / 1000 * 100);
    filedBuffer.push(graphics);
    fieldGeometryLayer.addChild(graphics);
  });

  geometryData.field.fieldArcs?.forEach((arc) => {
    const graphics = new Graphics();
    graphics.lineStyle(1, 'white', 1);
    graphics.arc(center.x  + arc.center!.x!  / 1000 * 100, center.y + arc.center!.y!  / 1000 * 100, arc.radius!  / 1000 * 100, 0, 2 * Math.PI);
    filedBuffer.push(graphics);
    fieldGeometryLayer.addChild(graphics);
  });
};

// Clear drawings when team changes
watch(() => gameStore.team, () => {
  drawings.yellowRobots.forEach((robotDrawing, _) => robotDrawing.destroy());
  drawings.blueRobots.forEach((robotDrawing, _) => robotDrawing.destroy());
  drawings.yellowRobots.clear()
  drawings.blueRobots.clear()
});

const updateRobotDrawing = (drawingsMap: Map<number, RobotDrawing>, robot: IWorldRobot, team: 'yellow' | 'blue') => {
  const robotId = robot.id ?? -1;
  if (!drawingsMap.has(robotId)) {
    const drawing = new RobotDrawing({
      canvasCenter: center,
      team: team,
      text: gameStore.team == team ? robotId.toString() : undefined,
    });

    movingObjectsLayer.addChild(drawing);
    drawingsMap.set(robotId, drawing);
  }
  const drawing = drawingsMap.get(robotId)!;
  drawing.moveOnField(flipSide.value.x * robot.pos!.x!, flipSide.value.y * robot.pos!.y!, robot.angle ?? 0);
}

onUpdated(async () => {

  app = new Application({
    width: props.size.width * 1.15,
    height: props.size.height * 1.15,
    backgroundColor: CanvasSettings.backgroundColor,
    view: canvas.value!
  });


  center = {x: app.screen.width / 2, y: app.screen.height / 2};
  drawings.ball = new BallDrawing(center);
  movingObjectsLayer.addChild(drawings.ball!);

  app.stage.addChild(fieldGeometryLayer);
  app.stage.addChild(movingObjectsLayer);
  app.stage.addChild(cursor);

  app.ticker.add(() => {
    const world = worldStore.latest?.lastSeenWorld;
    if (world === null || world === undefined) {return;}

    drawFiled(worldStore.latest!.field!);

    for (let robot of world.yellow!) {
      updateRobotDrawing(drawings.yellowRobots, robot, 'yellow');
    }

    for (let robot of world.blue!) {
      updateRobotDrawing(drawings.blueRobots, robot, 'blue');
    }
    // drawFiled(worldStore.latest!.field!);
    drawings.ball.moveOnField(flipSide.value.x * world!.ball!.pos!.x!, flipSide.value.y * world.ball!.pos!.y!);
  });

  app.stage.interactive = true;
  app.stage.hitArea = app.screen;

  app.stage.addEventListener('pointerleave', (e) => {cursor.text = "";});
  app.stage.addEventListener('pointermove', (e) => {
    cursor.position.copyFrom(e.global);
    cursor.text = `[${((center.x - e.data.global.x) / 100).toFixed(2)}x, ${((center.y - e.data.global.y) / 100).toFixed(2)}y]`

  });


});

onUnmounted(async () => {
  // listener();
  app?.destroy();
});

</script>
<template>
  <canvas class="min-m-6 m-auto min-h-0 min-w-0 max-h-full max-w-full w-auto h-auto rounded-xl"  ref="canvas"></canvas>
</template>
