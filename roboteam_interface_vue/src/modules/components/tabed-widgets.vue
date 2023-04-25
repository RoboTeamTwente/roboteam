<script setup lang="ts">
import {ref} from "vue";
import GameSettings from "./game-settings.vue";
import StpStatus from "./stp-status.vue";
import Feedback from "./feedback.vue";
import PlaysHistogram from "./plays-histogram.vue";
import Metrics from "./metrics.vue";
import UiSettings from "./ui-settings.vue";

const activeTab = ref(0);
</script>

<template>
  <div class="flex flex-col max-h-full">
    <div class="tabs rounded-none flex-nowrap overflow-auto whitespace-nowrap min-h-8">
      <a class="tab tab-bordered gap-1 flex-nowrap" @click="activeTab = 0" :class="{'tab-active': activeTab === 0}">
        <font-awesome-icon icon="fa-gear"/>
        Game Setting
      </a>
      <a class="tab tab-bordered gap-1 flex-nowrap" @click="activeTab = 1" :class="{'tab-active': activeTab === 1}">
        <font-awesome-icon icon="fa-layer-group"/>
        STP
      </a>
      <a class="tab tab-bordered gap-1 flex-nowrap" @click="activeTab = 2" :class="{'tab-active': activeTab === 2}">
        <font-awesome-icon icon="fa-rss"/>
        Feedback
      </a>
      <a class="tab tab-bordered gap-1 flex-nowrap" @click="activeTab = 3" :class="{'tab-active': activeTab === 3}">
        <font-awesome-icon icon="fa-history"/>
        Plays History
      </a>
      <a class="tab tab-bordered gap-1 flex-nowrap" @click="activeTab = 4" :class="{'tab-active': activeTab === 4}">
        <font-awesome-icon icon="fa-gauge"/>
        Metrics
      </a>
      <a class="tab tab-bordered gap-1 flex-nowrap" @click="activeTab = 5" :class="{'tab-active': activeTab ===5}">
        <font-awesome-icon icon="fa-gear"/>
        UI Setting
      </a>
      <div class="tab tab-bordered gap-1 flex-nowrap w-full " />
    </div>
    <div class="p-2 overflow-auto">
      <game-settings v-if="activeTab === 0"/>
      <stp-status v-if="activeTab === 1"/>
      <feedback v-if="activeTab === 2"/>
      <plays-histogram v-if="activeTab === 3"/>
      <metrics v-if="activeTab === 4"/>
      <ui-settings v-if="activeTab === 5"/>
    </div>
  </div>
</template>