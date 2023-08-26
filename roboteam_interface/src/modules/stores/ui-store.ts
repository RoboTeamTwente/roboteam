import { defineStore } from 'pinia'
import { robotNameMap } from '../../utils'
import { TabKey, TABS_DEFINITION } from '../../tabs'

type Panel = {
  size: number
  collapsed: boolean
  selectedTab: TabKey
  selectableTabs: TabKey[]
}

export type TriState = 'SHOW' | 'HIDE' | 'FOR_SELECTED_ROBOTS'

export type UiStore = {
  bottomPanel: Panel
  leftPanel: Panel
  selectedRobots: Set<number>,
  pointerLocation: {
    x: number
    y: number
  } | null,
  scaling: {
    ball: number
    robots: number
  }
  visualizations: {
    velocities: TriState
    pathPlanning: TriState
    debug: TriState
  }
  internalTeam: 'BLACK' | 'PURPLE'
}

const defaultState: () => UiStore = () => ({
  bottomPanel: {
    size: 250,
    collapsed: false,
    selectedTab: 'Game Settings',
    selectableTabs: Object.keys(TABS_DEFINITION) as TabKey[]
  },
  leftPanel: {
    size: 250,
    collapsed: false,
    selectedTab: 'STP',
    selectableTabs: Object.keys(TABS_DEFINITION) as TabKey[]
  },
  selectedRobots: new Set([]),
  pointerLocation: null,
  scaling: {
    ball: 1,
    robots: 1
  },
  visualizations: {
    velocities: 'SHOW',
    drawings: 'SHOW',
    pathPlanning: 'SHOW',
    debug: 'SHOW'
  },
  internalTeam: 'PURPLE'
})

export const useUIStore = defineStore('uiStore', {
  persist: {
    serializer: {
      serialize: JSON.stringify,
      deserialize: (value: string) => {
        const parsed = JSON.parse(value)
        // TODO: This is a hack to make the Set deserializable.
        parsed.selectedRobots = new Set()
        return parsed
      }
    }
  },
  state: () => defaultState(),
  actions: {
    togglePanel(panel: 'bottomPanel' | 'leftPanel') {
      this[panel].collapsed = !this[panel].collapsed
    },
    resizePanel(panel: 'bottomPanel' | 'leftPanel', size: number) {
      this[panel].size = size < defaultState()[panel].size ? defaultState()[panel].size : size
    },
    toggleRobotSelection(id: number) {
      this.selectedRobots.has(id) ? this.selectedRobots.delete(id) : this.selectedRobots.add(id)
    }
  },
  getters: {
    panelSize(state) {
      return (panel: 'bottomPanel' | 'leftPanel') =>
        state[panel].collapsed ? 0 : state[panel].size
    },
    isaRobotSelected(state) {
      return (id: number) => state.selectedRobots.has(id)
    },
    showDebug(state) {
      return (forRobot?: number | null) =>
        state.visualizations.debug === 'SHOW' ||
        (state.visualizations.debug === 'FOR_SELECTED_ROBOTS' &&
          state.selectedRobots.has(forRobot ?? -1))
    },
    showPathPlanning(state) {
      return (forRobot?: number | null) =>
        state.visualizations.pathPlanning === 'SHOW' ||
        (state.visualizations.pathPlanning === 'FOR_SELECTED_ROBOTS' &&
          state.selectedRobots.has(forRobot ?? -1))
    },
    showVelocities(state) {
      return (forRobot?: number | null) =>
        state.visualizations.velocities === 'SHOW' ||
        (state.visualizations.velocities === 'FOR_SELECTED_ROBOTS' &&
          state.selectedRobots.has(forRobot ?? -1))
    },
    robotName(state) {
      return (id: number) => robotNameMap(state.internalTeam, id)
    }
  }
})
