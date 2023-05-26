import {computed, ref, shallowRef, ShallowRef} from "vue";
import {useWebSocket, UseWebSocketOptions} from "@vueuse/core";
import {proto} from "./generated/proto";
import MsgToInterface = proto.MsgToInterface;

export type DeepReadonly<T> = T extends Function ? T : T extends object ? { readonly [K in keyof T]: DeepReadonly<T[K]> } : T;
export type ShallowReadonlyRef<T> = ShallowRef<DeepReadonly<T>>;

export const sleep = (time: number) => {
    return new Promise((resolve) => setTimeout(resolve, time));
}

export const robotNameMap = (team: 'BLACK' | 'PURPLE', id: number) => {
    if (team === 'PURPLE') {
        return {
            1: "Wall-E",
            2: "R2D2",
            3: "Tron",
            4: "Marvin",
            5: "Jarvis",
            6: "Baymax",
            7: "Noo-Noo",
            8: "T-800",
            9: "K-9",
            10: "Bender",
            11: "Holt",
            12: "Chappie",
            13: "TARS",
            14: "",
            15: "Herman",
        }[id]
    }

    return ""
};

export const useProtoWebSocket = () => {
    const status = ref<"CLOSED" | "OPENED" | "OPENING">("CLOSED");
    const wsRef = ref<WebSocket | undefined>();
    const protoData = shallowRef<MsgToInterface | null>(null);

    const onMessage = async (event: MessageEvent) => {
        const messageBuffer = new Uint8Array(await event.data.arrayBuffer());
        protoData.value = proto.MsgToInterface.decode(messageBuffer);
    };

    const sendProtoMsg = (properties?: proto.IMsgFromInterface) => {
        const buffer = proto.MsgFromInterface.encode(
            proto.MsgFromInterface.create(properties)
        ).finish();
        wsRef.value?.send(buffer.buffer.slice(buffer.byteOffset, buffer.byteOffset + buffer.length));
    };

    const open = (url: string) => {
        if (wsRef.value !== undefined) {
            // Status code 1000 -> Normal Closure https://developer.mozilla.org/en-US/docs/Web/API/CloseEvent/code
            wsRef.value.close(1000);
        }

        const ws = new WebSocket(url);
        wsRef.value = ws;
        status.value = "OPENING";

        ws.onmessage = onMessage;
        ws.onopen = () => status.value = "OPENED";
        ws.onclose = () => {
            status.value = "CLOSED"
            wsRef.value = undefined;
        };
    }

    return {
        data: protoData as ShallowRef<proto.MsgToInterface>,
        status,
        open,
        send: sendProtoMsg,
    }
}