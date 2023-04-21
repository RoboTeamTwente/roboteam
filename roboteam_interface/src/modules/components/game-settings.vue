<script setup lang="ts">

import {useGameSettingsStore} from "../stores/game-settings-store";
const gameStore = useGameSettingsStore();

</script>
<template>
  <div class="grid grid-cols-fluid-12 gap-x-8 gap-y-4">
    <div class="form-control">
      <span class="">General</span>
      <label class="label cursor-pointer gap-2">
        <span class="label-text">AutoRef</span>
        <input type="checkbox" v-model="gameStore.useReferee" class="toggle toggle-primary" checked/>
      </label>
      <label class="label cursor-pointer gap-2">
        <span class="label-text">Ignore Invariants</span>
        <input type="checkbox" v-model="gameStore.ignoreInvariants" class="toggle toggle-primary" checked/>
      </label>
    </div>

    <div class="form-control">
      <span class="">Receiver</span>
      <div class="form-control">
        <label class="label cursor-pointer">
          <span class="label-text"><font-awesome-icon icon="fa-vr-cardboard"/> Simulator</span>
          <input type="radio" class="radio" v-model="gameStore.hubMode" value="simulator" />
        </label>
      </div>
      <div class="form-control">
        <label class="label cursor-pointer">
          <span class="label-text"><font-awesome-icon icon="fa-robot"/> Basestation</span>
          <input type="radio" class="radio" v-model="gameStore.hubMode" value="basestation" />
        </label>
      </div>
    </div>

    <div class="form-control">
      <span class="">Side</span>
      <div class="form-control">
        <label class="label cursor-pointer">
          <span class="label-text"><font-awesome-icon icon="fa-arrow-left"/> Left</span>
          <input type="radio" class="radio" v-model="gameStore.side" value="left" />
        </label>
      </div>
      <div class="form-control">
        <label class="label cursor-pointer">
          <span class="label-text"><font-awesome-icon icon="fa-arrow-right"/> Right</span>
          <input type="radio"  class="radio" v-model="gameStore.side" value="right" />
        </label>
      </div>
    </div>

    <div class="form-control">
      <span class="">Team</span>
      <div class="form-control">
        <label class="label cursor-pointer">
          <span class="label-text"><font-awesome-icon icon="fa-people-group" class="text-warning"/> Yellow</span>
          <input type="radio" class="radio radio-warning" v-model="gameStore.team" value="yellow" />
        </label>
      </div>
      <div class="form-control">
        <label class="label cursor-pointer">
          <span class="label-text"><font-awesome-icon icon="fa-people-group" class="text-primary"/> Blue</span>
          <input type="radio" class="radio radio-primary" v-model="gameStore.team" value="blue" />
        </label>
      </div>
    </div>
  </div>
</template>