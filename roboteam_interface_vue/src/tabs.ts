import {markRaw} from "vue";
import GameSettings from "./modules/components/game-settings.vue";
import StpStatus from "./modules/components/panels/stp-panel/stp-status.vue";
import Feedback from "./modules/components/panels/feedback.vue";
import PlayEvaluation from "./modules/components/panels/play-evaluation.vue";
import Metrics from "./modules/components/panels/metrics.vue";
import UiSettings from "./modules/components/ui-settings/ui-settings.vue";

export const TABS_DEFINITION= {
    'Game Settings': {
        icon: 'fa-gear',
        component: markRaw(GameSettings)
    },
    'STP': {
        icon: 'fa-layer-group',
        component: markRaw(StpStatus)
    },
    'Feedback': {
        icon: 'fa-rss',
        component: markRaw(Feedback)
    },
    'Plays History': {
        icon: 'fa-history',
        component: markRaw(PlayEvaluation)
    },
    'Metrics': {
        icon: 'fa-gauge',
        component: markRaw(Metrics)
    },
    'UI Settings': {
        icon: 'fa-gear',
        component: markRaw(UiSettings)
    }
}

export type TabKey = keyof typeof TABS_DEFINITION;