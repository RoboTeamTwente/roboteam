import {useGameControllerStore} from "../modules/stores/ai-store";
import {watch} from "vue";

import {useVisualizationStore} from "../modules/stores/dataStores/visualization-store";
import {useSTPDataStore} from "../modules/stores/dataStores/stp-data-store";
import {useVisionDataStore} from "../modules/stores/dataStores/vision-data-store";
import {useProtoWebSocket} from "../utils";
import {emitter} from "../services/ai-events";
import {useAIDataStore} from "../modules/stores/dataStores/ai-data-store";


export const useAIClient = () => {
    // const gameSettingsStore = useGameSettingsStore();
    const gameControllerStore = useGameControllerStore();
    const stpDataStore = useSTPDataStore();
    const visionDataStore = useVisionDataStore();
    const visualizationStore = useVisualizationStore();

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
                stpDataStore.processSTPMsg(message.stpStatus!);
                break;
            case 'state':
                visionDataStore.processVisionMsg(message.state!);
                break;
            case 'visualizations':
                visualizationStore.pushDrawings(message.visualizations!.drawings!);
                visualizationStore.pushMetrics(message.visualizations!.metrics!);
                break;
            default:
                console.log('unknown message', message);
        }
    });

    emitter.on('update:runtimeConfiguration', (config) => {
        send({setRuntimeConfig: config})
    });

    emitter.on('update:gameSettings', (config) => {
        send({setGameSettings: config})
    });

    emitter.on('update:pause', (value) => {
        send({pauseAi: value})
    });

    emitter.on('update:play', (value) => {
        console.log('update:play', value);
        send({setPlay: value})
    });

    return {
        open,
        status,
    }
};