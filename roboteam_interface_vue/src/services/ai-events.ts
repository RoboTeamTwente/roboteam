import mitt, { Emitter } from 'mitt';
import {proto} from "../generated/proto";

export type Events = {
    'update:runtimeConfiguration': proto.RuntimeConfig,
    'update:gameSettings': proto.GameSettings,
    'update:pause': boolean,
    'update:play': proto.PlayInfo,
};

export const emitter: Emitter<Events> = mitt<Events>();