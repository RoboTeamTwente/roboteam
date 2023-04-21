import {defineStore} from 'pinia'

const defaultState: UiStore = {
    bottomPanel: {
        size: 250,
        collapsed: false
    },
    leftPanel: {
        size: 250,
        collapsed: false
    }
};

type Panel = {
    size: number
    collapsed: boolean
}

export type UiStore = {
    bottomPanel: Panel
    leftPanel: Panel
}
export const useUIStore = defineStore('uiStore', {
    persist: true,
    state: () => structuredClone(defaultState),
    actions: {
        toggleBottomPanel() {
            this.bottomPanel.collapsed = !this.bottomPanel.collapsed;
        },
        toggleLeftPanel() { this.leftPanel.collapsed = !this.leftPanel.collapsed; },
        resizeBottomPanel(size: number) {this.bottomPanel.size = size < defaultState.bottomPanel.size ? defaultState.bottomPanel.size : size},
        resizeLeftPanel(size: number) {this.leftPanel.size = size < defaultState.leftPanel.size ? defaultState.leftPanel.size : size}
    },
    getters: {
        bottomPanelSize(state) {
            return state.bottomPanel.collapsed ? 0 : state.bottomPanel.size;
        },
        leftPanelSize(state) {
            return state.leftPanel.collapsed ? 0 : state.leftPanel.size;
        }
    }
})