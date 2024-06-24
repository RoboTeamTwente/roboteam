<script setup lang="ts">
import { computed } from 'vue'
import { useUIStore } from './modules/stores/ui-store'
import TopPanel from './modules/components/layout/top-panel/top-panel.vue'
import TabedWidgets from './modules/components/layout/tabed-widgets.vue'
import { useVisionDataStore } from './modules/stores/data-stores/vision-data-store'
import SidebarResizer from './modules/components/layout/sidebar-resizer.vue'
import ConnectModal from './modules/components/connect-modal.vue'
import { useAiController } from './modules/composables/ai-controller'
import PixiApp from './modules/components/canvas/pixi-app.vue'
import FieldLines from './modules/components/canvas/field-lines.vue'
import Ball from './modules/components/canvas/ball.vue'
import Robots from './modules/components/canvas/robots.vue'
import Visualizations from './modules/components/canvas/visualizations.vue'
import { formatFloat } from './utils'

const aiController = useAiController()
const uiStore = useUIStore()
const visionData = useVisionDataStore()

const leftPanelSize = computed(() => `${uiStore.panelSize('leftPanel')}px`)
const bottomPanelSize = computed(() => `${uiStore.panelSize('bottomPanel')}px`)
</script>

<template>
  <connect-modal :status="aiController.status" @connect="aiController.open" />
  <div class="scaffold h-screen w-screen">
    <top-panel />

    <!-- Left Sidebar START -->
    <sidebar-resizer
      direction="x"
      class="left-sidebar-resize"
      @drag="uiStore.resizePanel('leftPanel', uiStore.leftPanel.size - $event.x)"
    />
    <Transition name="left-sidebar" :duration="300">
      <div class="left-sidebar" v-if="!uiStore.$state.leftPanel.collapsed">
        <tabed-widgets
          v-model:active-tab="uiStore.leftPanel.selectedTab"
          :tabs="uiStore.leftPanel.selectableTabs"
        />
      </div>
    </Transition>
    <!-- Left Sidebar END -->

    <!-- Game Canvas START -->
    <main
      id="game-canvas-container"
      class="game-canvas flex flex-col justify-center bg-base-200 min-h-0 p-4 relative"
      :class="{
        '!justify-start': visionData.latestWorld == null
      }"
    >
      <div
        v-if="uiStore.pointerLocation"
        class="absolute top-4 left-4 badge badge-secondary font-mono"
      >
        x: {{ formatFloat(uiStore.pointerLocation?.x) }} y:
        {{ formatFloat(uiStore.pointerLocation?.y) }}
      </div>
      <pixi-app
        v-if="visionData.latestField !== null"
        :length="visionData.latestField.fieldLength"
        :width="visionData.latestField.fieldWidth"
      >
        <field-lines :field-geometry="visionData.latestField" :is-yellow="aiController.isYellow" />
        <ball />
        <robots />
        <visualizations />
        <!-- Display the battery percentages of all robots -->
        <div class="right-side">
          <div :style="{ color: '#ffff'}">
            Battery Levels
          </div>
        <p v-for="robot in visionData.ourRobots!">
            <div v-if="robot.feedbackInfo?.ballSensorIsWorking == null" :style="{ color: '#808080'}">
              {{robot.id}} : {{robot.feedbackInfo?.batteryLevel}}
            </div>
            <div v-if="robot.feedbackInfo?.batteryLevel < 21" :style="{ color: '#8a0000'}">
              {{robot.id}} : {{robot.feedbackInfo?.batteryLevel}}
            </div>
            <div v-if="robot.feedbackInfo?.batteryLevel < 23.1" :style="{ color: '#ffff00'}">
              {{robot.id}} : {{robot.feedbackInfo?.batteryLevel}}
            </div>
            <div v-else :style="{ color: '#00a01e'}">
              {{robot.id}} : {{robot.feedbackInfo?.batteryLevel}}
            </div>
        </p>
      </div>
      </pixi-app>
      <div v-else class="alert alert-warning justify-start">
        <font-awesome-icon icon="fa-circle-exclamation" />
        No field data
      </div>
    </main>
    <!-- Game Canvas END -->

    <!-- Bottom sidebar START -->
    <sidebar-resizer
      direction="y"
      class="bottom-sidebar-resize"
      @drag="uiStore.resizePanel('bottomPanel', uiStore.bottomPanel.size + $event.y)"
    />
    <Transition name="bottom-sidebar" :duration="300">
      <div class="bottom-sidebar" v-if="!uiStore.$state.bottomPanel.collapsed">
        <tabed-widgets
          v-model:active-tab="uiStore.bottomPanel.selectedTab"
          :tabs="uiStore.bottomPanel.selectableTabs"
        />
      </div>
    </Transition>
    <!-- Bottom sidebar END -->
  </div>
</template>

<style>
.scaffold {
  display: grid;
  grid-template-areas:
    'tpa   tpa   tpa   tpa'
    'lsb   lbr   gac   gac'
    'lsb   lbr   bbr   bbr'
    'lsb   lbr   bsb   bsb';

  grid-template-columns: min(90vw, v-bind(leftPanelSize)) 3px 1fr 1fr auto;
  grid-template-rows: auto 1fr 3px min(80vh, v-bind(bottomPanelSize)) auto;
}

.top-panel {
  grid-area: tpa;
}

.left-sidebar {
  grid-area: lsb;
}

.left-sidebar-resize {
  grid-area: lbr;
}

.game-canvas {
  grid-area: gac;
}

.bottom-sidebar {
  grid-area: bsb;
}

.right-side {
  position: absolute;
  top: 0;
  right: 0;
  padding: 20px;
  background-color: #303030;
}

.bottom-sidebar-resize {
  grid-area: bbr;
}

.scaffold:has(
    .left-sidebar-enter-active,
    .left-sidebar-leave-active,
    .bottom-sidebar-enter-active,
    .bottom-sidebar-leave-active
  ) {
  transition: all 400ms;
}
</style>
