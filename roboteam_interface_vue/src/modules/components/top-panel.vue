<script setup lang="ts">
import {useUIStore} from "../stores/ui-store";
import {haltPlayName, useAIStore} from "../stores/ai-store";
import {computed} from "vue";
import {useGameSettingsStore} from "../stores/game-settings-store";

const uiStore = useUIStore();
const aiStore = useAIStore();
const gameSettingsStore = useGameSettingsStore();

const disabled = computed(() => {
  return gameSettingsStore.useReferee
});

con

</script>
<template>
  <header class="grid-in-header bg-base flex justify-center border-b-2 border-base-300 items-center px-4">
    <div class="flex">
      <button class="btn btn-sm btn-ghost gap-2" :class="{'btn-active': !uiStore.$state.leftPanel.collapsed}"
              @click="uiStore.toggleLeftPanel">
        <font-awesome-icon icon="fa-table-columns" rotation="270"/>
      </button>
      <button class="btn btn-sm btn-ghost gap-2" :class="{'btn-active': !uiStore.$state.bottomPanel.collapsed}"
              @click="uiStore.toggleBottomPanel">
        <font-awesome-icon icon="fa-table-columns" rotation="180"/>
      </button>
    </div>
    <div class="flex grow"/>
    <div class="flex gap-4">
      <div class="btn-group">
        <button :class="{'btn-disabled': disabled}" class="btn btn-sm btn-error gap-2 w-32" v-if="aiStore.$state.running" @click="aiStore.setRunning(false)"><font-awesome-icon icon="fa-square" /> Stop</button>
        <button :class="{'btn-disabled': disabled}" class="btn btn-sm btn-success gap-2 w-32" v-else @click="aiStore.setRunning(true)"><font-awesome-icon icon="fa-play" /> Resume</button>
        <button :class="{'btn-disabled': disabled}" class="btn btn-sm btn-secondary gap-2" @click="aiStore.play = haltPlayName"><font-awesome-icon icon="fa-hand" /> Halt</button>
      </div>

      <div class="input-group w-auto">
        <select class="select select-sm select-bordered" v-model="aiStore.play" :disabled="disabled">
          <option v-for="play in aiStore.$state.availablePlays" :value="play">
            {{ play }}
          </option>
        </select>
        <select class="select select-sm select-bordered" v-model="aiStore.ruleset" :disabled="disabled">
          <option v-for="subPlay in aiStore.$state.availableRulesets" :value="subPlay">
            {{ subPlay }}
          </option>
        </select>
<!--        <select class="select select-sm select-bordered">-->
<!--          <option>Homer</option>-->
<!--          <option>Marge</option>-->
<!--          <option>Bart</option>-->
<!--          <option>Lisa</option>-->
<!--          <option>Maggie</option>-->
<!--        </select>-->
      </div>
      <div class="btn-group">
        <button class="btn btn-sm btn-primary gap-2" @click="aiStore.resetPlay" :class="{'btn-disabled': disabled}"><font-awesome-icon icon="fa-rotate-right" /> Reset Play</button>
      </div>
    </div>
    <div class="flex grow"/>
    <div class="flex">
      <div class="dropdown dropdown-end">
        <label tabindex="0" class="btn btn-circle btn-ghost btn-sm">
          <font-awesome-icon icon="fa-info-circle"/>
        </label>
        <div tabindex="0" class="card compact dropdown-content shadow bg-base-100 rounded-box w-64">
          <div class="card-body ">
            <h2 class="card-title">Performance Info</h2>
            <ul class="font-mono">
              <li>Tick: {{aiStore.$state.stpData.currentTick}}</li>
              <li>FPS: XXX</li>
            </ul>
          </div>
        </div>
      </div>

    </div>
  </header>
</template>


<style scoped>

</style>