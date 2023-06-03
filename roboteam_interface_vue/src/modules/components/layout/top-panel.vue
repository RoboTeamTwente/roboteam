<script setup lang="ts">
import {useUIStore} from "../../stores/ui-store";
import {computed, toRaw} from "vue";
import {useSTPDataStore} from "../../stores/data-stores/stp-data-store";
import {useAIDataStore} from "../../stores/data-stores/ai-data-store";
import {sleep} from "../../../utils";
import {useAiController} from "../../composables/ai-controller";
import {uiEmitter} from "../../../services/events";

const uiStore = useUIStore();
const stpData = useSTPDataStore();
const aiData = useAIDataStore();
const aiController = useAiController();
const disabled = computed(() => aiController.useReferee.value);

const haltPlay = () => {
    stpData.$state.currentPlayName = 'Halt';
}

const resetPlay = async () => {
    const tmp = toRaw(stpData.currentPlayName);
    stpData.$state.currentPlayName = 'Halt';
    await sleep(100);
    stpData.$state.currentPlayName = tmp;
}

const togglePause = () => {
    aiController.isPaused.value = !aiController.isPaused.value;
}

</script>
<template>
  <header class="top-panel bg-base flex justify-center border-b-2 border-base-300 items-center px-4">
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
        <button :class="{
            'btn-disabled': disabled,
            'btn-success': aiController.isPaused.value,
            'btn-error': !aiController.isPaused.value
        }" class="btn btn-sm gap-2 w-32" @click="togglePause">
            <template v-if="!aiController.isPaused.value"> <font-awesome-icon icon="fa-square" /> Pause </template>
            <template v-else> <font-awesome-icon icon="fa-play" /> Resume </template>
        </button>
        <button :class="{'btn-disabled': disabled}" class="btn btn-sm btn-secondary gap-2" @click="haltPlay"><font-awesome-icon icon="fa-hand" /> Halt</button>
      </div>

      <div class="input-group w-auto">
        <select class="select select-sm select-bordered" v-model="stpData.currentPlayName" :disabled="disabled">
          <option v-for="play in aiData.state!.plays" :value="play">
            {{ play }}
          </option>
        </select>
        <select class="select select-sm select-bordered" v-model="stpData.currentRuleset" :disabled="disabled">
          <option v-for="ruleset in aiData.state!.ruleSets" :value="ruleset">
            {{ ruleset }}
          </option>
        </select>
      </div>
      <div class="btn-group">
        <button class="btn btn-sm btn-primary gap-2" @click="resetPlay" :class="{'btn-disabled': disabled}"><font-awesome-icon icon="fa-rotate-right" /> Reset Play</button>
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
            <button class="btn btn-error btn-sm" @click="() => uiEmitter.emit('wss:disconnect')">Disconnect from AI</button>
            <h2 class="card-title">Performance Info</h2>
            <ul class="font-mono">
              <li>Tick: {{stpData.currentTick}}</li>
            </ul>
          </div>
        </div>
      </div>

    </div>
  </header>
</template>


<style scoped>

</style>