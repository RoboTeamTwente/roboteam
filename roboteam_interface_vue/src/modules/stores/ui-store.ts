import {defineStore} from 'pinia'
import {robotNameMap} from "../../utils";
import GameSettings from "../components/game-settings.vue";
import {Component, markRaw} from "vue";
import StpStatus from "../components/panels/stp-panel/stp-status.vue";
import Feedback from "../components/panels/feedback.vue";
import PlayEvaluation from "../components/panels/play-evaluation.vue";
import Metrics from "../components/panels/metrics.vue";
import UiSettings from "../components/ui-settings/ui-settings.vue";
import {s} from "@tauri-apps/api/shell-cbf4da8b";

type Panel = {
    size: number
    collapsed: boolean
    selectedTab: Tab,
    selectableTabs: Tab[]
}

export type TriState = 'SHOW' | 'HIDE' | 'FOR_SELECTED_ROBOTS'

export type UiStore = {
    bottomPanel: Panel
    leftPanel: Panel
    selectedRobots: Set<number>
    scaling: {
        ball: number,
        robots: number,
    },
    visualizations: {
        velocities: TriState,
        pathPlanning: TriState,
        debug: TriState,
    },
    internalTeam: 'BLACK' | 'PURPLE'
}

export type Tab = {
    name: string,
    icon: string,
    component: string
};

export const TABS: Tab[] = [
    {
        name: 'Game Settings',
        icon: 'fa-gear',
        component: markRaw(GameSettings)
    },
    {
        name: 'STP',
        icon: 'fa-layer-group',
        component: markRaw(StpStatus)
    },
    {
        name: 'Feedback',
        icon: 'fa-rss',
        component: markRaw(Feedback)
    },
    {
        name: 'Plays History',
        icon: 'fa-history',
        component: markRaw(PlayEvaluation)
    },
    {
        name: 'Metrics',
        icon: 'fa-gauge',
        component: markRaw(Metrics)
    },
    {
        name: 'UI Settings',
        icon: 'fa-gear',
        component: markRaw(UiSettings)
    }
];

const defaultState: () => UiStore = () => ({
    bottomPanel: {
        size: 250,
        collapsed: false,
        selectedTab: TABS[0],
        selectableTabs: TABS
    },
    leftPanel: {
        size: 250,
        collapsed: false,
        selectedTab: TABS[0],
        selectableTabs: TABS
    },
    selectedRobots: new Set([]),
    scaling: {
        ball: 1,
        robots: 1,
    },
    visualizations: {
        velocities: 'SHOW',
        drawings: 'SHOW',
        pathPlanning: 'SHOW',
        debug: 'SHOW',
    },
    internalTeam: 'PURPLE'
});


export const useUIStore = defineStore('uiStore', {
    persist: {
        serializer: {
            serialize: JSON.stringify,
            deserialize: (value: string) => {
                const parsed = JSON.parse(value);
                // TODO: This is a hack to make the Set deserializable.
                parsed.selectedRobots = new Set();
                return parsed;
            }
        }
    },
    state: () => defaultState(),
    actions: {
        toggleBottomPanel() {
            this.bottomPanel.collapsed = !this.bottomPanel.collapsed;
        },
        toggleLeftPanel() {
            this.leftPanel.collapsed = !this.leftPanel.collapsed;
        },
        resizeBottomPanel(size: number) {
            this.bottomPanel.size = size < defaultState().bottomPanel.size ? defaultState().bottomPanel.size : size
        },
        resizeLeftPanel(size: number) {
            this.leftPanel.size = size < defaultState().leftPanel.size ? defaultState().leftPanel.size : size
        },
        toggleRobotSelection(id: number) {
            this.selectedRobots.has(id)
                ? this.selectedRobots.delete(id)
                : this.selectedRobots.add(id);
        }
    },
    getters: {
        bottomPanelSize(state) {
            return state.bottomPanel.collapsed ? 0 : state.bottomPanel.size;
        },
        leftPanelSize(state) {
            return state.leftPanel.collapsed ? 0 : state.leftPanel.size;
        },
        isaRobotSelected(state) {
            return (id: number) => state.selectedRobots.has(id);
        },
        showDebug(state) {
            return (forRobot?: number | null) => state.visualizations.debug === 'SHOW' || (state.visualizations.debug === 'FOR_SELECTED_ROBOTS' && state.selectedRobots.has(forRobot ?? -1));
        },
        showPathPlanning(state) {
            return (forRobot?: number | null) => state.visualizations.pathPlanning === 'SHOW' || (state.visualizations.pathPlanning === 'FOR_SELECTED_ROBOTS' && state.selectedRobots.has(forRobot ?? -1));
        },
        showVelocities(state) {
            return (forRobot?: number | null) => state.visualizations.velocities === 'SHOW' || (state.visualizations.velocities === 'FOR_SELECTED_ROBOTS' && state.selectedRobots.has(forRobot ?? -1));
        },
        robotName(state) {
            return (id: number) => robotNameMap(state.internalTeam, id);
        }
    }
})