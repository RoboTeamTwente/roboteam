import {watch} from "vue";

import {useVisualizationStore} from "../modules/stores/data-stores/visualization-store";
import {useSTPDataStore} from "../modules/stores/data-stores/stp-data-store";
import {useVisionDataStore} from "../modules/stores/data-stores/vision-data-store";
import {useProtoWebSocket} from "../utils";
import {aiEmitter} from "./events";
import {useAIDataStore} from "../modules/stores/data-stores/ai-data-store";


export const useAIClient = () => {
    const stpData = useSTPDataStore();
    const visionData = useVisionDataStore();
    const visualizationData = useVisualizationStore();

    const aiData = useAIDataStore();
    // const {status, data, send} = useWebSocket(url, {autoReconnect: true});
    const {status, data, send, open} = useProtoWebSocket();

    // On message received
    watch(data, (message) => {
        switch (message.kind) {
            case 'aiState':
                aiData.updateStateFromProto(message.aiState!);
                break;
            case 'stpStatus':
                stpData.processSTPMsg(message.stpStatus!);
                break;
            case 'state':
                visionData.processVisionMsg(message.state!);
                break;
            case 'visualizations':
                visualizationData.pushDrawings(message.visualizations!.drawings!);
                visualizationData.pushMetrics(message.visualizations!.metrics!);
                break;
            default:
                console.log('unknown message', message);
        }
    });

    aiEmitter.on('update:runtimeConfiguration', (config) => {
        send({setRuntimeConfig: config})
    });

    aiEmitter.on('update:gameSettings', (config) => {
        console.log('update:gameSettings', config);
        send({setGameSettings: config})
    });

    aiEmitter.on('update:pause', (value) => {
        send({pauseAi: value})
    });

    aiEmitter.on('update:play', (value) => {
        send({setPlay: value})
    });

    aiEmitter.on('setBallPos', (value) => {
        console.log('setBallPos', value)
        send({setBallPos: value})
    });

    return {
        open,
        status,
    }
};