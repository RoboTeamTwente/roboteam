import {defineStore} from "pinia";
import {proto} from "../../../generated/proto";
import ISTPStatus = proto.ISTPStatus;
import {DeepReadonly} from "../../../utils";
import {computed, readonly, ref, shallowRef} from "vue";

export const useSTPDataStore = defineStore('stpDataStore', () => {
    // State
    const latest = shallowRef<ISTPStatus | null>(null)
    const currentTick = ref(-1)

    // Actions
    const processSTPMsg = (msg: proto.ISTPStatus) => {
        latest.value = msg;
        currentTick.value = msg.currentTick!;
    }

    const $reset = () => {
        latest.value = null;
        currentTick.value = 0;
    }

    return {
        latest: readonly(latest),
        currentTick: readonly(currentTick),
        processSTPMsg,
        $reset
    }
})