import {Container, FederatedPointerEvent, Graphics, Text} from "pixi.js";
import {proto} from "../../generated/proto";
import IVector2 = proto.IVector2;

type Team = 'yellow' | 'blue';

export const Colors = {
    yellow: '#feff00',
    blue: '#9793ff',
    robotSelected: 0x1aad1a,
    ball: 0xFF6600
}

export const CanvasSettings = {
    width: 1200,
    height: 900,
    backgroundColor: "#224922"
}

export type Size = {width: number, height: number};

export class RobotDrawing extends Container {
    readonly canvasCenter: IVector2;
    robotElem: Graphics;
    textElem?: Text;

    constructor({canvasCenter, team, text, onClick}:{canvasCenter: IVector2, team: Team, text?: string, onClick?: (self?: RobotDrawing, event?: FederatedPointerEvent) => void}) {
        super();
        this.canvasCenter = canvasCenter;

        // Robot drawing is always the first child
        this.robotElem = new Graphics()
            .beginFill(team === 'yellow' ? Colors.yellow : Colors.blue)
            // .beginFill(Colors.blueRobot)
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
        // console.log("toggleSelection", selected);
        // this.robotElem.tint = selected ? Colors.yellowRobot : Colors.robotSelected;
    }

    moveOnField(x: number, y: number, angle: number) {
        this.x = this.canvasCenter.x + x * 100;
        this.y = this.canvasCenter.y + y * 100;
        this.robotElem.angle = angle * 57;
    }

}

export class BallDrawing extends Graphics {
    readonly canvasCenter: IVector2;

    constructor(canvasCenter: IVector2) {
        super();
        this.canvasCenter = canvasCenter

        this.beginFill(0xFF6600)
        this.drawCircle(0, 0, 10)
        this.endFill()
    }

    moveOnField(x: number, y: number) {
        this.x = this.canvasCenter.x + x * 100;
        this.y = this.canvasCenter.y + y * 100;;
    }
}