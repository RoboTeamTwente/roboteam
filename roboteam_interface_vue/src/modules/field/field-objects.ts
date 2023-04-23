import {Container, FederatedPointerEvent, Graphics, SimpleRope, Sprite, Text, Texture} from "pixi.js";
import {proto} from "../../generated/proto";
import IDrawing = proto.IDrawing;
import {useGameSettingsStore} from "../stores/game-settings-store";
import {DeepReadonly, toRaw} from "vue";

import pixelWhite from '../../assets/pixel-white.png';
import dot from '../../assets/dot-16x16.png';

type Team = 'yellow' | 'blue';

export const Colors = {
    yellow: '#feff00',
    blue: '#9793ff',
    robotSelected: '#1aad1a',
    ball: '#FF6600',
    backgroundColor: '#224922'
};

export type Size = { width: number, height: number };
export type FieldColors = {
    leftGoal: string,
    rightGoal: string,
}

export const useFieldObjectStorage = () => {
    const drawings = {
        ball: new BallDrawing(),
        blueRobots: new Map<number, RobotDrawing>(),
        yellowRobots: new Map<number, RobotDrawing>(),
        shapes: new ShapeMap<string, ShapeDrawing>(),
    }

    const layers = {
        fieldGeometry: new Container(),
        movingObjects: new Container(),
        shapeLayer: new Container(),
    }

    const clearRobotsDrawings = () => {
        drawings.ball.destroy();
        drawings.ball = new BallDrawing();

        drawings.yellowRobots.forEach((robotDrawing, _) => robotDrawing.destroy());
        drawings.blueRobots.forEach((robotDrawing, _) => robotDrawing.destroy());
        drawings.yellowRobots.clear();
        drawings.blueRobots.clear();
    }

    return {
        clearRobotsDrawings,
        drawings,
        layers,
    }
};

const protoColorToHex = (color: proto.Drawing.Color): string => {
    switch (color) {
        case proto.Drawing.Color.RED:
            return '#ff0000';
        case proto.Drawing.Color.GREEN:
            return '#00ff00';
        case proto.Drawing.Color.BLUE:
            return '#0000ff';
        case proto.Drawing.Color.YELLOW:
            return '#ffff00';
        case proto.Drawing.Color.CYAN:
            return '#00ffff';
        case proto.Drawing.Color.MAGENTA:
            return '#ff00ff';
        case proto.Drawing.Color.WHITE:
            return '#ffffff';
        case proto.Drawing.Color.BLACK:
            return '#000000';
    }
}

const mmToPx = (mm: number): number => {
    return mm / 1000 * 100;
}

export class RobotDrawing extends Container {
    readonly originalColor: string;
    robotElem: Graphics;
    textElem?: Text;

    constructor({
                    team,
                    text,
                    onClick
                }: { team: Team, text?: string, onClick?: (self?: RobotDrawing, event?: FederatedPointerEvent) => void }) {
        super();
        this.originalColor = team === 'yellow' ? Colors.yellow : Colors.blue

        // Robot drawing is always the first child
        this.robotElem = new Graphics()
            .beginFill(this.originalColor)
            .arc(0, 0, 15, 0.86707957, -0.86707957)
            .endFill();
        this.addChild(this.robotElem);

        if (onClick !== undefined) {
            this.robotElem.eventMode = 'static';
            this.robotElem.cursor = 'pointer';
            this.robotElem.on('click', (event) => onClick(this, event));
        }

        if (text !== undefined) {
            this.textElem = this.addChild(new Text(text, {fontSize: 14}));
            this.textElem.anchor.set(0.5);
        }
    }

    toggleSelection(selected: boolean) {
        this.robotElem.tint = selected ? Colors.robotSelected : this.originalColor;
    }

    moveOnField(x: number, y: number, angle: number) {
        this.x = x * 100;
        this.y = y * 100;
        this.robotElem.angle = angle * 57;
    }

}

export class BallDrawing extends Graphics {

    constructor() {
        super();
        this.beginFill(Colors.ball)
        this.drawCircle(0, 0, 10)
        this.endFill()
    }

    moveOnField(x: number, y: number) {
        this.x = x * 100;
        this.y = y * 100;
    }
}

export class FieldDrawing extends Graphics {
    constructor({fieldGeometry, fieldColors}: { fieldGeometry: DeepReadonly<proto.ISSL_GeometryFieldSize>, fieldColors: FieldColors}) {
        super();
        fieldGeometry.fieldLines?.forEach((line) => {
            switch (line.name) {
                case 'LeftGoalDepthLine':
                    this.lineStyle(2, fieldColors.leftGoal);
                    break;
                case 'RightGoalDepthLine':
                    this.lineStyle(2, fieldColors.rightGoal);
                    break;
                default:
                    this.lineStyle(1, '#FFFFFF');
            }

            this.moveTo(mmToPx(line.p1!.x!), mmToPx(line.p1!.y!))
                .lineTo(mmToPx(line.p2!.x!), mmToPx(line.p2!.y!));
        });

        fieldGeometry.fieldArcs?.forEach((arc) => {
            this.lineStyle(1, 'white', 1)
                .moveTo(mmToPx(arc.center!.x!), mmToPx(arc.center!.y!))
                .arc(mmToPx(arc.center!.x!), mmToPx(arc.center!.y!), mmToPx(arc.radius!), 0, 2 * Math.PI);
        });
    }
}

export class ShapeDrawing extends Container {
    readonly retainUntilTick: number;

    constructor({data, currentTick}: { data: IDrawing, currentTick: number }) {
        super();
        this.retainUntilTick = currentTick + data.retainForTicks!;

        const orientation = toRaw(useGameSettingsStore().fieldOrientation);
        const points = data.points!.map((point) => {
            return {
                x: -orientation.x * point.x! * 100,
                y: -orientation.y * point.y! * 100
            }
        });

        switch (data.method!) {
            case proto.Drawing.Method.LINES_CONNECTED:
                const lineTexture = Texture.from(pixelWhite);
                const strip = new SimpleRope(lineTexture, points, 4);
                strip.tint = protoColorToHex(data.color!);
                this.addChild(strip);
                break;
            case proto.Drawing.Method.DOTS:
                const dotTexture = Texture.from(dot);
                points.forEach((point) => {
                    const dot = new Sprite(dotTexture);
                    dot.width = 6;
                    dot.height = 6;
                    dot.tint = protoColorToHex(data.color!);
                    dot.position.copyFrom(point);
                    this.addChild(dot);
                });
                break;
        }
    }
}

export class ShapeMap<K, V extends ShapeDrawing> extends Map<K, V> {
    set(key: K, value: V): this {
        if (this.has(key)) {
            this.get(key)!.destroy();
        }

        super.set(key, value);
        return this;
    }

    removeExpiredShapes(currentTick: number): void {
        this.forEach((value, key) => {
            if (currentTick < value.retainUntilTick) {
                return;
            }

            value.destroy();
            this.delete(key);
        });
    }
}