<script setup lang="ts">
import {
  Application,
  Container,
  Text,
} from "pixi.js";
import {onMounted, onUnmounted, ref, toRaw, watch} from "vue";
import {
  BallDrawing,
  Colors,
  FieldDrawing,
  RobotDrawing,
  ShapeDrawing,
  initFieldObjectStorage
} from "./FieldObjects";
import {useWorldStateStore} from "../stores/world-store";
import {useGameSettingsStore} from "../stores/game-settings-store";
import {proto} from "../../generated/proto";
import IWorldRobot = proto.IWorldRobot;
import {useUIStore} from "../stores/ui-store";
import {useVisualizationStore} from "../stores/visualization-store";

const props = defineProps<{
  field: proto.ISSL_GeometryFieldSize;
}>();

const canvas = ref<HTMLCanvasElement | null>(null);
const worldStore = useWorldStateStore();
const gameStore = useGameSettingsStore();
const uiStore = useUIStore();
const visualizationStore = useVisualizationStore();

let app: null | Application = null;
let {layers, drawings} = initFieldObjectStorage();

// Clear drawings when team changes
watch(() => gameStore.team, () => {
  drawings.yellowRobots.forEach((robotDrawing, _) => robotDrawing.destroy());
  drawings.blueRobots.forEach((robotDrawing, _) => robotDrawing.destroy());
  drawings.yellowRobots.clear()
  drawings.blueRobots.clear()

  layers.fieldGeometry.removeChildren(0).forEach((child) => child.destroy());
  layers.fieldGeometry.addChild(new FieldDrawing({
    fieldGeometry: props.field,
    fieldColors: toRaw(gameStore.goalColor),
  }));
});

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

    layers.movingObjects.addChild(drawing);
    drawingsMap.set(robotId, drawing);
  }
  const drawing = drawingsMap.get(robotId)!;
  drawing.toggleSelection(toRaw(uiStore.isaRobotSelected(robotId)) && team == gameStore.team);
  drawing.moveOnField(gameStore.fieldOrientation.x * robot.pos!.x!, gameStore.fieldOrientation.y * robot.pos!.y!, gameStore.fieldOrientation.angle + (-robot.angle ?? 0));
}

onMounted(async () => {
  app = new Application({
    width: worldStore.latest?.field?.field?.fieldLength! / 10 * 1.15,
    height: worldStore.latest?.field?.field?.fieldWidth! / 10 * 1.15,
    backgroundColor: Colors.backgroundColor,
    view: canvas.value!
  });

  // Init field geometry drawings
  layers.fieldGeometry.addChild(new FieldDrawing({
    fieldGeometry: props.field,
    fieldColors: toRaw(gameStore.goalColor),
  }));

  // Init cursor position text
  const cursor = new Text("", {fontSize: 16, fill: 'white'});
  cursor.x = app.screen.width * 0.025;
  cursor.y = app.screen.height * 0.025;

  // Init ball drawing
  drawings.ball = new BallDrawing();
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
  app.stage.interactive = true;
  app.stage.hitArea = app.screen;
  app.stage.addEventListener('pointerleave', () => {cursor.text = "";});
  app.stage.addEventListener('pointermove', (e) => {
    const pos = e.getLocalPosition(container);
    cursor.text = `[${((pos.x) / 100).toFixed(2)}x, ${((pos.y) / 100).toFixed(2)}y]`
  });

  // Update the translation and projection matrices every frame
  app.ticker.add(() => {
    const world = worldStore.latest?.lastSeenWorld;
    if (world === null || world === undefined) {return;}

    // Update robot drawings
    world.yellow!.forEach(robot => updateRobotDrawing(drawings.yellowRobots, robot, 'yellow'))
    world.blue!.forEach(robot => updateRobotDrawing(drawings.blueRobots, robot, 'blue'))

    // Update ball drawing
    drawings.ball.moveOnField(gameStore.fieldOrientation.x * world!.ball!.pos!.x!, gameStore.fieldOrientation.y * world.ball!.pos!.y!);

    // Update shape drawings
    drawings.shapes.removeExpiredShapes();
    visualizationStore
        .popAll()
        .forEach(props => {
            const shape = new ShapeDrawing({data: props});
            layers.shapeLayer.addChild(shape);
            drawings.shapes.set(props.label!, shape); // Drawings map will automatically call destroy on the old shape
    });
  });
});

onUnmounted(async () => app?.destroy());

</script>
<template>
  <canvas class="min-m-6 m-auto min-h-0 min-w-0 max-h-full max-w-full w-auto h-auto rounded-xl"  ref="canvas"></canvas>
</template>
