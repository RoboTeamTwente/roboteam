import {defineStore} from 'pinia'
import {toRaw} from "vue";

const defaultState: UiStore = {
    bottomPanel: {
        size: 250,
        collapsed: false
    },
    leftPanel: {
        size: 250,
        collapsed: false
    },
    selectedRobots: new Set([])
};

type Panel = {
    size: number
    collapsed: boolean
}

export type UiStore = {
    bottomPanel: Panel
    leftPanel: Panel
    selectedRobots: Set<number>
}
export const useUIStore = defineStore('uiStore', {
    persist: {
        serializer: {
            serialize: JSON.stringify,
            deserialize: (value: string) => {
                const parsed = JSON.parse(value);
                parsed.selectedRobots = new Set(parsed.selectedRobots);
                return parsed;
            }
        }
    },
    state: () => defaultState,
    actions: {
        toggleBottomPanel() {this.bottomPanel.collapsed = !this.bottomPanel.collapsed;},
        toggleLeftPanel() { this.leftPanel.collapsed = !this.leftPanel.collapsed; },
        resizeBottomPanel(size: number) {this.bottomPanel.size = size < defaultState.bottomPanel.size ? defaultState.bottomPanel.size : size},
        resizeLeftPanel(size: number) {this.leftPanel.size = size < defaultState.leftPanel.size ? defaultState.leftPanel.size : size},
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
        }
    }
})