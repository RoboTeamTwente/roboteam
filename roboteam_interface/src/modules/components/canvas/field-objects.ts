import {
  Application,
  Container,
  FederatedPointerEvent,
  Graphics,
  SimpleRope,
  Text,
  Texture
} from 'pixi.js'
import IDrawing = proto.IDrawing
import { DeepReadonly, toRaw } from 'vue'

// @ts-ignore
import pixel from '../../../assets/pixel-white.png'
import IWorldRobot = proto.IWorldRobot
import { IApplicationOptions } from '@pixi/app/lib/Application'
import { proto } from '../../../generated/proto'
import { useAIDataStore } from '../../stores/data-stores/ai-data-store'
import { useSTPDataStore } from '../../stores/data-stores/stp-data-store'
import { useUIStore } from '../../stores/ui-store'
import { NoUndefinedField } from '../../../utils'

export const Colors = {
  yellow: '#ffff00',
  blue: '#0000ff',
  robotSelected: '#9a1c74',
  ball: '#FF6600',
  backgroundColor: '#00a01e',
  fieldLines: '#ffffff'
}

export type Size = { width: number; height: number }
export type FieldColors = {
  leftGoal: string
  rightGoal: string
}

const protoColorToHex = (color: proto.Drawing.Color): string => {
  switch (color) {
    case proto.Drawing.Color.RED:
      return '#ff0000'
    case proto.Drawing.Color.GREEN:
      return '#00ff00'
    case proto.Drawing.Color.BLUE:
      return '#0000ff'
    case proto.Drawing.Color.YELLOW:
      return '#ffff00'
    case proto.Drawing.Color.CYAN:
      return '#00ffff'
    case proto.Drawing.Color.MAGENTA:
      return '#ff00ff'
    case proto.Drawing.Color.WHITE:
      return '#ffffff'
    case proto.Drawing.Color.GREY:
      return '#666666'  
    case proto.Drawing.Color.BLACK:
      return '#000000'
  }
}

const mmToPx = (mm: number): number => {
  return (mm / 1000) * 100
}

export class CustomPixiApplication extends Application {
  readonly centeredContainer: Container
  layers: {
    fieldLines: Container
    objects: Container
    drawings: Container
    ball: Container
  }

  constructor(options?: Partial<IApplicationOptions>) {
    super(options)

    this.centeredContainer = new Container()

    // this puts the (0, 0) coordinates to the center of the stage
    this.centeredContainer.x = this.screen.width / 2
    this.centeredContainer.y = this.screen.height / 2
    this.stage.addChild(this.centeredContainer)

    this.layers = {
      fieldLines: new Container(),
      objects: new Container(),
      drawings: new Container(),
      ball: new Container()
    }

    // order matters
    this.centeredContainer.addChild(this.layers.fieldLines)
    this.centeredContainer.addChild(this.layers.objects)
    this.centeredContainer.addChild(this.layers.drawings)
    this.centeredContainer.addChild(this.layers.ball)
  }
}

export class RobotDrawing extends Container {
  readonly originalColor: string
  robotElem: Graphics
  robotOutline: Graphics
  velocityMeter: Graphics
  textElem?: Text

  constructor({
    isYellow,
    text,
    onClick
  }: {
    isYellow: boolean
    text?: string
    onClick?: (self?: RobotDrawing, event?: FederatedPointerEvent) => void
  }) {
    super()
    this.originalColor = isYellow ? Colors.yellow : Colors.blue

    this.velocityMeter = new Graphics().lineStyle(4, Colors.fieldLines, 0).lineTo(100, 100)
    this.addChild(this.velocityMeter)

    this.robotOutline = new Graphics()

    this.robotElem = new Graphics()
      .beginFill(this.originalColor)
      .arc(0, 0, 0.089 * 100, 0.86707957, -0.86707957)
      .endFill()
    this.addChild(this.robotElem)

    if (onClick !== undefined) {
      this.eventMode = 'static'
      this.cursor = 'pointer'
      this.on('click', (event) => onClick(this, event))
    }

    if (text !== undefined) {
      const textColor = isYellow ? 0x000000 : 0xffffff
      this.textElem = this.addChild(new Text(text, { fontSize: 14, fill: textColor }))
      this.textElem.anchor.set(0.5)
    }
  }

  update(
    isSelected: boolean,
    showVelocity: boolean,
    fieldOrientation: {
      x: number
      y: number
      yaw: number
    },
    data: IWorldRobot
  ) {
    const robots = useSTPDataStore()?.latest?.robots;
    const uiStore = useUIStore();
    let outlineColor : string | undefined;

    this.robotOutline
    .clear()

    if (robots && uiStore.showRobotRoles()) {
      Object.values(robots).forEach((robot) => {
        if (robot.id === data.id && showVelocity) {
          switch (robot.role?.name) {
            case 'harasser':
              outlineColor = '#8B0000'
              break
            case 'passer':
            case 'striker':
            case "ball_placer":
            case "kicker":
            case "kicker_formation":
            case "free_kick_taker":
            case "kick_off_taker":
              outlineColor = Colors.ball
              break
            case 'receiver':
              outlineColor = '#ff00ff'
              break 
          }
          if (outlineColor !== undefined) {
            this.robotOutline
            .lineStyle(4, outlineColor)
            .arc(0, 0, 0.089 * 150 + 0.089 * uiStore.scaling.robots, 0.86707957, -0.86707957)
            this.addChild(this.robotOutline)
          }
        }
      })
    }

    this.toggleSelection(isSelected)
    this.moveOnField(
      fieldOrientation.x * data.pos!.x!,
      fieldOrientation.y * data.pos!.y!,
      fieldOrientation.yaw + -(data.yaw ?? 0)
    )
    this.updateVelocityDrawing(
      showVelocity,
      fieldOrientation.x * data.vel!.x!,
      fieldOrientation.y * data.vel!.y!
    )
  }

  toggleSelection(selected: boolean) {
    this.robotElem.tint = selected ? Colors.robotSelected : this.originalColor
  }

  moveOnField(x: number, y: number, yaw: number) {
    this.x = x * 100
    this.y = y * 100
    this.robotElem.angle = yaw * 57
    this.robotOutline.angle = yaw * 57
  }

  updateVelocityDrawing(showVelocity: boolean, x: number, y: number) {
    this.velocityMeter.clear()
    if (showVelocity) {
      this.velocityMeter.lineStyle(1, Colors.fieldLines, 1).lineTo(x * 100, y * 100)
    }
  }
}

export class BallDrawing extends Graphics {
  constructor() {
    super()
    this.beginFill(Colors.ball)
    this.drawCircle(0, 0, 0.0215 * 100)
    this.endFill()
  }

  moveOnField(x: number, y: number) {
    this.x = x * 100
    this.y = y * 100
  }
}

export class FieldDrawing extends Graphics {
  constructor({
    fieldGeometry,
    isYellow
  }: {
    fieldGeometry: DeepReadonly<proto.ISSL_GeometryFieldSize>
    isYellow: boolean
  }) {
    super()

    const fieldColors = isYellow
      ? { leftGoal: Colors.yellow, rightGoal: Colors.blue }
      : { leftGoal: Colors.blue, rightGoal: Colors.yellow }
    const kGoalWidth = fieldGeometry.goalWidth
    const kGoalDepth = fieldGeometry.goalDepth
    const kXMax = fieldGeometry.fieldLength / 2

    const derivedLines = [
      {
        name: 'LeftGoalTopLine',
        p1: { x: -kXMax, y: kGoalWidth / 2 },
        p2: { x: -kXMax - kGoalDepth, y: kGoalWidth / 2 }
      },
      {
        name: 'LeftGoalBottomLine',
        p1: { x: -kXMax, y: -kGoalWidth / 2 },
        p2: { x: -kXMax - kGoalDepth, y: -kGoalWidth / 2 }
      },
      {
        name: 'LeftGoalDepthLine',
        p1: { x: -kXMax - kGoalDepth, y: -kGoalWidth / 2 },
        p2: { x: -kXMax - kGoalDepth, y: kGoalWidth / 2 }
      },
      {
        name: 'RightGoalTopLine',
        p1: { x: kXMax, y: kGoalWidth / 2 },
        p2: { x: kXMax + kGoalDepth, y: kGoalWidth / 2 }
      },
      {
        name: 'RightGoalBottomLine',
        p1: { x: kXMax, y: -kGoalWidth / 2 },
        p2: { x: kXMax + kGoalDepth, y: -kGoalWidth / 2 }
      },
      {
        name: 'RightGoalDepthLine',
        p1: { x: kXMax + kGoalDepth, y: -kGoalWidth / 2 },
        p2: { x: kXMax + kGoalDepth, y: kGoalWidth / 2 }
      }
    ]

    const allLines = [
      ...(Array.isArray(derivedLines) ? derivedLines : []),
      ...(Array.isArray(fieldGeometry.fieldLines) ? fieldGeometry.fieldLines : [])
    ]
    allLines.forEach((line) => {
      switch (line.name) {
        case 'LeftGoalDepthLine':
        case 'LeftGoalTopLine':
        case 'LeftGoalBottomLine':
          this.lineStyle(4, fieldColors.leftGoal)
          break
        case 'RightGoalDepthLine':
        case 'RightGoalTopLine':
        case 'RightGoalBottomLine':
          this.lineStyle(4, fieldColors.rightGoal)
          break
        default:
          this.lineStyle(2, Colors.fieldLines)
      }

      this.moveTo(mmToPx(line.p1.x), mmToPx(line.p1.y)).lineTo(mmToPx(line.p2.x), mmToPx(line.p2.y))
    })

    fieldGeometry.fieldArcs?.forEach((arc) => {
      this.lineStyle(2, Colors.fieldLines, 1)
        .moveTo(mmToPx(arc.center!.x!), mmToPx(arc.center!.y!))
        .arc(mmToPx(arc.center!.x!), mmToPx(arc.center!.y!), mmToPx(arc.radius!), 0, 2 * Math.PI)
    })
  }
}

export class ShapeDrawing extends Container {
  readonly retainUntilTick: number

  constructor({ data, currentTick }: { data: NoUndefinedField<IDrawing>; currentTick: number }) {
    super()
    this.retainUntilTick = currentTick + data.retainForTicks!

    const orientation = toRaw(useAIDataStore().fieldOrientation)
    const points = data.points!.map((point) => {
      return {
        x: -orientation.x * point.x! * 100,
        y: -orientation.y * point.y! * 100
      }
    })

    if (data.method! === proto.Drawing.Method.LINES_CONNECTED) {
      const lineTexture = Texture.from(pixel)
      // @ts-ignore Vector2[] is compatible with IPoint[] shape
      const strip = new SimpleRope(lineTexture, points, data.thickness)
      strip.tint = protoColorToHex(data.color!)
      this.addChild(strip)
      return
    }

    const graphicsPrototype = new Graphics()
    switch (data.method) {
      case proto.Drawing.Method.DOTS:
        graphicsPrototype.beginFill(protoColorToHex(data.color!))
        graphicsPrototype.drawCircle(0, 0, data.size)
        break
      case proto.Drawing.Method.PLUSES:
        graphicsPrototype.lineStyle(data.thickness, protoColorToHex(data.color!))
        graphicsPrototype.drawPolygon([
          { x: -data.size, y: 0 },
          { x: data.size, y: 0 },
          { x: 0, y: 0 },
          { x: 0, y: -data.size },
          { x: 0, y: data.size },
          { x: 0, y: 0 }
        ])
        break
      case proto.Drawing.Method.CROSSES:
        graphicsPrototype.lineStyle(data.thickness, protoColorToHex(data.color!))
        graphicsPrototype.drawPolygon([
          { x: -data.size, y: -data.size },
          { x: data.size, y: data.size },
          { x: 0, y: 0 },
          { x: data.size, y: -data.size },
          { x: -data.size, y: data.size },
          { x: 0, y: 0 }
        ])
        break
      case proto.Drawing.Method.CIRCLES:
        graphicsPrototype.lineStyle(data.thickness, protoColorToHex(data.color!))
        graphicsPrototype.drawCircle(0, 0, data.size)
        break
    }
    points.forEach((point) => {
      const graphics = graphicsPrototype.clone()
      graphics.x = point.x
      graphics.y = point.y
      this.addChild(graphics)
    })
  }
}

export class ShapeMap<K, V extends ShapeDrawing> extends Map<K, V> {
  set(key: K, value: V): this {
    if (this.has(key)) {
      this.get(key)!.destroy()
    }

    super.set(key, value)
    return this
  }

  removeExpiredShapes(currentTick: number): void {
    this.forEach((value, key) => {
      if (currentTick < value.retainUntilTick) {
        return
      }

      value.destroy({ children: true, texture: true, baseTexture: true })
      this.delete(key)
    })
  }
}
