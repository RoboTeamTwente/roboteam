import {defineStore} from "pinia";
import {proto} from "../../generated/proto";
import ISTPStatus = proto.ISTPStatus;

type StpStoreState = {
    latest: ISTPStatus | null
}

export const useSTPStore = defineStore('stpStore', {
    state: (): StpStoreState => {return {
        latest: null,
    }},
    actions: {
        pushNewStatus(msg: ISTPStatus) {
            this.latest = msg;
        }
    },
});