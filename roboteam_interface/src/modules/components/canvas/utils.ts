import { InjectionKey, Ref, ShallowRef } from 'vue'
import { Container } from '@pixi/display'
import { DisplayObject, FederatedPointerEvent, IPoint } from 'pixi.js'
import { CustomPixiApplication } from './field-objects'
import { useEventListener, useKeyModifier, useMousePressed, useThrottleFn } from '@vueuse/core'
import { useAiController } from '../../composables/ai-controller'
import { useUIStore } from '../../stores/ui-store'
import { useAIDataStore } from '../../stores/data-stores/ai-data-store'
import { proto } from '../../../generated/proto'
import { useVisionDataStore } from '../../stores/data-stores/vision-data-store'
import { useMagicKeys } from '@vueuse/core'

export const appSymbol = Symbol() as InjectionKey<ShallowRef<CustomPixiApplication>>
export const stageSymbol = Symbol() as InjectionKey<ShallowRef<Container<DisplayObject>>>

const transformCoordinates = (point: IPoint) => {
  return { x: point.x / 100, y: -point.y / 100 }
}

export const zoom = (factor: number, x: number, y: number, stage: Container) => {
  factor = factor > 0 ? 1.10 : 0.90

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


export const useMoveCamera = (canvasRef: Ref<HTMLCanvasElement | null>, appRef: ShallowRef<CustomPixiApplication | null>) => {
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
  app: ShallowRef<CustomPixiApplication>,
  stage: ShallowRef<Container<DisplayObject>>
) => {
  const
    aiController = useAiController(),
    keys = useMagicKeys(),
    { pressed } = useMousePressed()

  const onMouseMove = useThrottleFn((e: FederatedPointerEvent) => {
    if (!(keys.b.value || keys.shift.value) || !pressed.value) return
    const pos = transformCoordinates(e.getLocalPosition(app.value!.layers.objects))
    aiController.sendSimulatorCommand({
      control: { teleportBall: { x: pos.x, y: pos.y, z: 0.0, vx: 0.0, vy: 0.0, vz: 0.0 } }
    })
  }, 50)

  useEventListener(stage, 'pointermove', onMouseMove)
}

export const useShootBall = (
  app: ShallowRef<CustomPixiApplication>,
  stage: ShallowRef<Container<DisplayObject>>
) => {
  const
    aiController = useAiController(),
    keys = useMagicKeys(),
    { pressed } = useMousePressed()
  const onClick = useThrottleFn((e: FederatedPointerEvent) => {
    let speed = 6;
    let numberKeyPressed = false;
    for (let i = 1; i <= 9; i++) {
        if (keys[i.toString()].value) {
            speed = i;
            numberKeyPressed = true;
            break;
        }
    }
    if (!numberKeyPressed && (!keys.s.value || !pressed.value)) return
    const pos = transformCoordinates(e.getLocalPosition(app.value!.layers.objects))
    const visionData = useVisionDataStore()
    const world = visionData.latestWorld
    if (!world) return
    const ball = world.ball
    if (!ball || !ball.pos || !ball.pos.x || !ball.pos.y) return
    const dx = pos.x - ball.pos.x
    const dy = pos.y - ball.pos.y
    const distance = Math.sqrt(dx * dx + dy * dy)
    const time = distance / speed
    const vx = dx / time
    const vy = dy / time
    aiController.sendSimulatorCommand({
      control: { teleportBall: { x: ball.pos.x, y: ball.pos.y, z: 0.0, vx: vx, vy: vy, vz: 0.0 } }
    })
  }, 50)
  useEventListener(stage, 'click', onClick)
}

export const useMoveRobots = (
  app: ShallowRef<CustomPixiApplication>,
  stage: ShallowRef<Container<DisplayObject>>
) => {
  const
    aiController = useAiController(),
    uiStore = useUIStore(),
    aiData = useAIDataStore(),
    keys = useMagicKeys(),
    { pressed } = useMousePressed()

  const onClick = useThrottleFn((e: FederatedPointerEvent) => {
    if (!(keys.r.value || keys.alt.value) || !pressed.value) return

    const pos = transformCoordinates(e.getLocalPosition(app.value!.layers.objects))

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

  useEventListener(stage, 'click', onClick)
  useEventListener(stage, 'pointermove', onClick)
}

export const usePointerLocation = (
  stage: ShallowRef<Container<DisplayObject> | null>,
  app: ShallowRef<CustomPixiApplication | null>
) => {
  const uiStore = useUIStore()

  useEventListener(stage, 'pointermove', (e: FederatedPointerEvent) => {
    if (!stage.value) return
    const pos = e.getLocalPosition(app.value?.centeredContainer!)
    uiStore.pointerLocation = transformCoordinates(pos)
  });

  useEventListener(stage, 'pointerleave', () => {
    uiStore.pointerLocation = null
  });
}