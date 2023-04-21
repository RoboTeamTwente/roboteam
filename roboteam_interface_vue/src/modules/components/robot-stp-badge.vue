<script setup lang="ts">
import {computed} from "vue";
import {proto} from "../../generated/proto";
import Status = proto.STPStatus.STPRobot.Status;

export type Skill =
    | 'Chip'
    | 'Go To Position'
    | 'Rotate'
    | 'Orbit'
    | 'Kick'
    | 'Orbit Angular';

export type Role =
    | 'Attacker'
    | 'Ball Placer'
    | 'Chipper'
    | 'Free Kick Taker'
    | 'Harasser'
    | 'Keeper Passer'
    | 'Passer'
    | 'Pass Receiver'
    | 'passive'
    | 'Ball Avoider'
    | 'Ball Defender'
    | 'Formation'
    | 'Halt'
    | 'Robot Defender'
    | 'Keeper'
    | 'Penalty Keeper'
    | 'Test Role'

export type Tactics =
    | 'Chip At Position'
    | 'Drive With Ball'
    | 'Get Ball'
    | 'Get Behind Ball In Direction'
    | 'Kick At Pos'
    | 'Orbit Kick'
    | 'Receive'
    | 'Avoid Ball'
    | 'Ball Stand Back'
    | 'Block Ball'
    | 'Block Robot'
    | 'Formation'
    | 'Halt'
    | 'Keeper Block Ball'
    | 'Test Tactic';

// export type Status =
//     | 'Finished'
//     | 'Running'
//     | 'Error'
//     | 'Waiting';


const props = defineProps<{
  status: Status,
  name: Skill | Role | Tactics
}>();

const statusIcon = computed(() => {
  return {
    1: "fa-circle-check",
    3: 'fa-arrows-rotate',
    2: 'fa-triangle-exclamation',
    0: 'fa-hourglass-start',
  }[props.status] || 'fa-question';

})

const icon = computed(() => {
  return {
    'Chip': "fa-arrow-up-right",
    'Go To Position': 'fa-arrow-right',
    'Rotate': 'fa-undo',
    'Orbit': 'fa-circle',
    'Kick': 'fa-futbol',
    'Orbit Angular': 'fa-circle',
    'Attacker': 'fa-crosshairs',
    'Ball Placer': 'fa-bullseye',
    'Chipper': 'fa-arrow-up-right',
    'Free Kick Taker': 'fa-futbol',
    'Harasser': 'fa-hand-rock',
    'Keeper Passer': 'fa-hands-helping',
    'Passer': 'fa-long-arrow-alt-right',
    'Pass Receiver': 'fa-circle',
    'passive': 'fa-dizzy',
    'Ball Avoider': 'fa-stop',
    'Ball Defender': 'fa-shield-alt',
    'Robot Defender': 'fa-robot',
    'Keeper': 'fa-grip-vertical',
    'Penalty Keeper': 'fa-grin-beam-sweat',
    'Test Role': 'fa-flask',
    'Get Behind Ball In Direction': "fa-chevron-circle-down",
    'Chip At Position': 'fa-arrow-up-right',
    'Drive With Ball': 'fa-forward',
    'Get Ball': 'fa-dot-circle',
    'Kick At Pos': 'fa-futbol',
    'Orbit Kick': 'fa-circle',
    'Receive': 'fa-circle',
    'Avoid Ball': 'fa-exclamation-triangle',
    'Ball Stand Back': 'fa-reply',
    'Block Ball': 'fa-hand-paper',
    'Block Robot': 'fa-shield-alt',
    'Formation': 'fa-sitemap',
    'Halt': 'fa-stop-circle',
    'Keeper Block Ball': 'fa-hand-rock',
    'Test Tactic': 'fa-cogs'
  } [props.name] || 'fa-hashtag'

});

</script>

<template>
  <div class="kbd gap-2 text-sm md:text-base flex justify-between">
    <font-awesome-icon :icon="icon" class="w-3 h-3"/>
    {{ props.name }}
    <font-awesome-icon :icon="statusIcon" :spin="props.status === 3" class="w-3 h-3 text-secondary" :class="{
        'text-success': props.status === 1,
        'text-error': props.status === 2,
    }"/>
  </div>
</template>

