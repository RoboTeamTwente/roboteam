<script setup lang="ts">
import { appSymbol, stageSymbol, useMoveRobots } from './utils'
import { useVisionDataStore } from '../../stores/data-stores/vision-data-store'
import { useAIDataStore } from '../../stores/data-stores/ai-data-store'
import { useUIStore } from '../../stores/ui-store'
import { OUT_OF_CANVAS_COORDINATES } from '../../../utils'
import { proto } from '../../../generated/proto'
import IWorldRobot = proto.IWorldRobot
import { inject, onBeforeUnmount, watch } from 'vue'
import { RobotDrawing } from './field-objects'

const app = inject(appSymbol)!,
  stage = inject(stageSymbol)!,
  visionData = useVisionDataStore(),
  aiData = useAIDataStore(),
  uiStore = useUIStore()

// Internal (non-reactive) variables
let yellowRobots = new Map<number, RobotDrawing>(),
  blueRobots = new Map<number, RobotDrawing>()

useMoveRobots(app, stage)

// Methods
const cleanup = () => {
    yellowRobots.forEach((robot) => robot.destroy({ children: true }))
    blueRobots.forEach((robot) => robot.destroy({ children: true }))
    yellowRobots.clear()
    blueRobots.clear()
    app.value?.ticker.remove(onPixiTick)
  },
  renderRobot = (robot: IWorldRobot, isYellow: boolean) => {
    const robotId = robot.id ?? -1
    const isUs = isYellow == aiData.state!.gameSettings?.isYellow // Only allow clicking on robots of your own team
    const fieldOrientation = aiData.fieldOrientation
    const drawings = isYellow ? yellowRobots : blueRobots

    if (!drawings.has(robotId)) {
      const drawing = new RobotDrawing({
        isYellow: isYellow,
        text: robotId.toString(),
        onClick: isUs ? () => uiStore.toggleRobotSelection(robotId) : undefined
      })

      drawing.scale.set(uiStore.scaling.robots)
      app.value?.layers.objects.addChild(drawing)
      drawings.set(robotId, drawing)
    }

    drawings
      .get(robotId)!
      .update(isUs && uiStore.isaRobotSelected(robotId), isUs, fieldOrientation, robot)
  },
  onPixiTick = () => {
    const world = visionData.latestWorld
    // Move all robots off the field, so that robots that are no longer in the world are hidden
    yellowRobots.forEach((robot) => {
      robot.moveOnField(
        OUT_OF_CANVAS_COORDINATES.x,
        OUT_OF_CANVAS_COORDINATES.y,
        OUT_OF_CANVAS_COORDINATES.yaw
      )
    })
    blueRobots.forEach((robot) => {
      robot.moveOnField(
        OUT_OF_CANVAS_COORDINATES.x,
        OUT_OF_CANVAS_COORDINATES.y,
        OUT_OF_CANVAS_COORDINATES.yaw
      )
    })

    world?.yellow!.forEach((robot) => renderRobot(robot, true))
    world?.blue!.forEach((robot) => renderRobot(robot, false))
  }

watch(
  app,
  (app, _, onCleanup) => {
    app?.ticker.add(onPixiTick)
    onCleanup(cleanup)
  },
  { immediate: true }
)

// When the scaling setting changes, update the scaling of the drawings
watch(
  () => uiStore.scaling.robots,
  () => {
    blueRobots.forEach((robot) => robot.scale.set(uiStore.scaling.robots))
    yellowRobots.forEach((robot) => robot.scale.set(uiStore.scaling.robots))
  }
)

onBeforeUnmount(cleanup)
</script>

<template>
  <div hidden>
    <!--  Robots Component (Nothing to render, rendering is done using pixi not Vue) -->
  </div>
</template>
