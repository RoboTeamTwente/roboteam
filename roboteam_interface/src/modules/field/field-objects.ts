import {
  Application,
  Container,
  FederatedPointerEvent,
  Graphics,
  SimpleRope,
  Text,
  Texture
} from 'pixi.js'
import { proto } from '../../generated/proto'
import IDrawing = proto.IDrawing
import { useAIDataStore } from '../stores/data-stores/ai-data-store'
import { DeepReadonly, toRaw } from 'vue'

// @ts-ignore
import pixel from '../../assets/pixel-white.png'
import IWorldRobot = proto.IWorldRobot
import { IApplicationOptions } from '@pixi/app/lib/Application'
import { NoUndefinedField } from '../../utils'

export const Colors = {
  yellow: '#feff00',
  blue: '#9793ff',
  robotSelected: '#1aad1a',
  ball: '#FF6600',
  backgroundColor: '#224922',
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
    case proto.Drawing.Color.BLACK:
      return '#000000'
  }
}

const mmToPx = (mm: number): number => {
  return (mm / 1000) * 100
}

export class CustomPixiApplication extends Application {
  drawingsContainer: Container

  constructor(options?: Partial<IApplicationOptions>) {
    super(options)
    this.drawingsContainer = new Container()

    // this puts the (0, 0) coordinates to the center of the stage
    this.drawingsContainer.x = this.screen.width / 2
    this.drawingsContainer.y = this.screen.height / 2
    this.stage.addChild(this.drawingsContainer)
  }
}

export class RobotDrawing extends Container {
  readonly originalColor: string
  robotElem: Graphics
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
      this.textElem = this.addChild(new Text(text, { fontSize: 14 }))
      this.textElem.anchor.set(0.5)
    }
  }

  update(
    isSelected: boolean,
    showVelocity: boolean,
    fieldOrientation: {
      x: number
      y: number
      angle: number
    },
    data: IWorldRobot
  ) {
    this.toggleSelection(isSelected)
    this.moveOnField(
      fieldOrientation.x * data.pos!.x!,
      fieldOrientation.y * data.pos!.y!,
      fieldOrientation.angle + -(data.angle ?? 0)
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

  moveOnField(x: number, y: number, angle: number) {
    this.x = x * 100
    this.y = y * 100
    this.robotElem.angle = angle * 57
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

    fieldGeometry.fieldLines?.forEach((line) => {
      switch (line.name) {
        case 'LeftGoalDepthLine':
          this.lineStyle(4, fieldColors.leftGoal)
          break
        case 'RightGoalDepthLine':
          this.lineStyle(4, fieldColors.rightGoal)
          break
        default:
          this.lineStyle(2, Colors.fieldLines)
      }

      this.moveTo(mmToPx(line.p1!.x!), mmToPx(line.p1!.y!)).lineTo(
        mmToPx(line.p2!.x!),
        mmToPx(line.p2!.y!)
      )
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
