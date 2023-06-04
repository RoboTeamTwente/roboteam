<script setup lang="ts">
import { Container } from 'pixi.js'
import { CustomPixiApplication, RobotDrawing } from './field-objects'
import { onUnmounted, watch } from 'vue'
import { useVisionDataStore } from '../stores/data-stores/vision-data-store'
import { proto } from '../../generated/proto'
import IWorldRobot = proto.IWorldRobot
import { useUIStore } from '../stores/ui-store'
import { useAIDataStore } from '../stores/data-stores/ai-data-store'

// Internal (non-reactive) variables
let layer: Container | null = new Container(),
  yellowRobots = new Map<number, RobotDrawing>(),
  blueRobots = new Map<number, RobotDrawing>()

// Reactive values
const props = defineProps<{
    app: CustomPixiApplication
  }>(),
  visionData = useVisionDataStore(),
  uiStore = useUIStore(),
  aiData = useAIDataStore()

// Methods
const init = () => {
    layer = new Container()
    props.app.drawingsContainer.addChild(layer)
    props.app.ticker.add(onPixiTick)
  },
  cleanUp = () => {
    console.log('Cleaning up robot drawings')
    layer?.destroy({ children: true })
    layer = null
    yellowRobots.clear()
    blueRobots.clear()

    props.app.ticker.remove(onPixiTick)
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
      layer?.addChild(drawing)
      drawings.set(robotId, drawing)
    }

    drawings
      .get(robotId)!
      .update(isUs && uiStore.isaRobotSelected(robotId), isUs, fieldOrientation, robot)
  },
  onPixiTick = () => {
    const world = visionData.latestWorld
    world?.yellow!.forEach((robot) => renderRobot(robot, true))
    world?.blue!.forEach((robot) => renderRobot(robot, false))
  }

// When the scaling setting changes, update the scaling of the drawings
watch(
  () => uiStore.scaling.robots,
  () => {
    blueRobots.forEach((robot) => robot.scale.set(uiStore.scaling.robots))
    yellowRobots.forEach((robot) => robot.scale.set(uiStore.scaling.robots))
  }
)

// When the container or team color changes, re-initialize the drawings
watch(
  [() => props.app, () => aiData.state?.gameSettings?.isYellow],
  () => {
    cleanUp()
    init()
  },
  { immediate: true }
)

onUnmounted(cleanUp)
</script>

<template>
  <div hidden>
    <!--  Robots Component (Nothing to render, rendering is done using pixi not Vue) -->
  </div>
</template>
