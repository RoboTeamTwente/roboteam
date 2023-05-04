import {defineStore} from "pinia";
import {proto} from "../../../generated/proto";
import ISTPStatus = proto.ISTPStatus;
import {DeepReadonly} from "../../../utils";
import {computed, readonly, ref, shallowRef} from "vue";

export const useSTPDataStore = defineStore('stpDataStore', () => {
    // State
    const latest = shallowRef<DeepReadonly<ISTPStatus | null>>(null)
    const currentTick = ref(-1)

    // Actions
    const processSTPMsg = (msg: proto.ISTPStatus) => {
        latest.value = msg as DeepReadonly<ISTPStatus>;
        currentTick.value = -1;
    }

    return {
        latest,
        currentTick,
        processSTPMsg
    }
})