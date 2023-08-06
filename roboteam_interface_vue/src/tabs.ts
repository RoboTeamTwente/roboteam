import { markRaw } from 'vue'
import GameSettings   from './modules/components/game-settings.vue'
import Simulator      from './modules/components/panels/simulator-widget.vue'
import StpStatus      from './modules/components/panels/stp-panel/stp-widget.vue'
import Feedback       from './modules/components/panels/feedback-widget.vue'
import PlayEvaluation from './modules/components/panels/play-evaluation-widget.vue'
import Metrics        from './modules/components/panels/metrics-widget.vue'
import UiSettings     from './modules/components/ui-settings/ui-settings.vue'

export const TABS_DEFINITION = {
  'Game Settings': {
    icon: 'fa-gear',
    component: markRaw(GameSettings)
  },
  'Simulator': {
    icon: 'fa-gamepad',
    component: markRaw(Simulator)
  },
  STP: {
    icon: 'fa-layer-group',
    component: markRaw(StpStatus)
  },
  Feedback: {
    icon: 'fa-rss',
    component: markRaw(Feedback)
  },
  'Plays History': {
    icon: 'fa-history',
    component: markRaw(PlayEvaluation)
  },
  Metrics: {
    icon: 'fa-gauge',
    component: markRaw(Metrics)
  },
  'UI Settings': {
    icon: 'fa-gear',
    component: markRaw(UiSettings)
  }
}

export type TabKey = keyof typeof TABS_DEFINITION
