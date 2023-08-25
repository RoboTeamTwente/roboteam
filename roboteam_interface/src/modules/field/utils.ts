import { Container } from '@pixi/display'
import { CustomPixiApplication } from './field-objects'
import { computed, Raw, ref, Ref, ShallowRef, watch } from 'vue'
import { useEventListener, useKeyModifier, useMousePressed, useThrottleFn } from '@vueuse/core'
import { DisplayObject, FederatedPointerEvent, IPoint } from 'pixi.js'
import { useAiController } from '../composables/ai-controller'
import { proto } from '../../generated/proto'
import { useUIStore } from '../stores/ui-store'
import { useAIDataStore } from '../stores/data-stores/ai-data-store'

const transformCoordinates = (point: IPoint) => {
  return { x: point.x / 100, y: -point.y / 100 }
}

export const zoom = (factor: number, x: number, y: number, stage: Container) => {
  factor = factor > 0 ? 2 : 0.5

  const worldPos = {
    x: (x - stage.x) / stage.scale.x,
    y: (y - stage.y) / stage.scale.y
  }

  const newScale = {
    x: stage.scale.x * factor,
    y: stage.scale.y * factor
  }

  const newScreenPos = { x: (worldPos.x) * newScale.x + stage.x, y: (worldPos.y) * newScale.y + stage.y }

  stage.x -= (newScreenPos.x - x)
  stage.y -= (newScreenPos.y - y)
  stage.scale.x = newScale.x
  stage.scale.y = newScale.y
}


export const useFieldZoom = (canvasRef: Ref<HTMLCanvasElement | null>, appRef: ShallowRef<CustomPixiApplication | null>) => {
  const ctrlKey = useKeyModifier('Control')
  let lastPos: null | { x: number, y: number } = null

  useEventListener(canvasRef, 'wheel', (e: WheelEvent) => {
    if (!ctrlKey.value) return
    if (appRef.value?.stage === null) return
    zoom(e.deltaY, e.offsetX, e.offsetY, appRef.value!.stage!)
  })

  useEventListener(canvasRef, 'mouseup', () => lastPos = null)
  useEventListener(canvasRef, 'mousedown', (e: MouseEvent) => lastPos = { x: e.offsetX, y: e.offsetY })
  useEventListener(canvasRef, 'mousemove', (e: MouseEvent) => {
    if (!ctrlKey.value) return
    if (appRef.value?.stage === null) return
    if (lastPos === null) return

    appRef.value!.stage!.x += (e.offsetX - lastPos.x)
    appRef.value!.stage!.y += (e.offsetY - lastPos.y)
    lastPos = { x: e.offsetX, y: e.offsetY }
  })
}

export const useMoveBall = (
  canvasRef: Ref<HTMLCanvasElement | null>,
  appRef: ShallowRef<CustomPixiApplication | null>,
  stage: Ref<Container<DisplayObject> | undefined>
) => {
  const
    aiController = useAiController(),
    shiftKey = useKeyModifier('Shift'),
    { pressed } = useMousePressed()

  const onMouseMove = useThrottleFn((e: FederatedPointerEvent) => {
    if (!shiftKey.value || !pressed.value) return
    const pos = transformCoordinates(e.getLocalPosition(appRef.value!.drawingsContainer))
    aiController.sendSimulatorCommand({
      control: { teleportBall: { ...pos } }
    })
  }, 50)

  useEventListener(stage, 'pointermove', onMouseMove)
}

export const useMoveRobots = (
  canvasRef: Ref<HTMLCanvasElement | null>,
  appRef: ShallowRef<CustomPixiApplication | null>,
  stage: Ref<Container<DisplayObject> | undefined>
) => {
  const
    aiController = useAiController(),
    uiStore = useUIStore(),
    aiData = useAIDataStore(),
    shiftKey = useKeyModifier('Alt'),
    { pressed } = useMousePressed()

  const onMouseMove = useThrottleFn((e: FederatedPointerEvent) => {
    if (!shiftKey.value || !pressed.value) return
    const pos = transformCoordinates(e.getLocalPosition(appRef.value!.drawingsContainer))

    const robots = [...uiStore.selectedRobots].map(id => ({
      id: { id: id, 'team': aiData.state!.gameSettings!.isYellow ? proto.Team.YELLOW : proto.Team.BLUE },
      orientation: 0,
      ...pos
    }))

    aiController.sendSimulatorCommand({
      control: {
        teleportRobot: robots
      }
    })
  }, 50)

  useEventListener(stage, 'pointermove', onMouseMove)
}