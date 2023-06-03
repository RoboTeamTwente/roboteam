import mitt, { Emitter } from 'mitt';
import {proto} from "../generated/proto";

export type AIEvents = {
    'update:runtimeConfiguration': proto.RuntimeConfig,
    'update:gameSettings': proto.GameSettings,
    'update:pause': boolean,
    'update:play': proto.PlayInfo,
    'setBallPos': proto.IVector2f,
};

export const aiEmitter: Emitter<AIEvents> = mitt<AIEvents>();


export type UIEvents = {
    'wss:disconnect': void,
};

export const uiEmitter: Emitter<UIEvents> = mitt<UIEvents>();