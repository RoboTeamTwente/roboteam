import { ShallowRef } from 'vue'

export type NoUndefinedField<T> = { [P in keyof T]-?: NoUndefinedField<NonNullable<T[P]>> }
export type DeepReadonly<T> = T extends Function
  ? T
  : T extends object
  ? { readonly [K in keyof T]: DeepReadonly<T[K]> }
  : T
export type ShallowReadonlyRef<T> = ShallowRef<DeepReadonly<T>>

export const sleep = (time: number) => {
  return new Promise((resolve) => setTimeout(resolve, time))
}

export const OUT_OF_CANVAS_COORDINATES = {
  x: 100,
  y: 100,
  angle: 0
}

export const FORMATION_1 = {
  "control": {
    "teleportBall": {
      "x": "0.000",
      "y": "0.000",
      "z": "0.021",
      "vx": "0.000",
      "vy": "0.000",
      "vz": "0.000"
    },
    "teleportRobot": [
      {
        "id": {
          "id": 0,
          "team": 2
        },
        "x": "-1.500",
        "y": "1.120",
        "orientation": "0.000"
      },
      {
        "id": {
          "id": 1,
          "team": 2
        },
        "x": "-1.500",
        "y": "-0.000",
        "orientation": "0.000"
      },
      {
        "id": {
          "id": 2,
          "team": 2
        },
        "x": "-1.500",
        "y": "-1.120",
        "orientation": "0.000"
      },
      {
        "id": {
          "id": 3,
          "team": 2
        },
        "x": "-0.550",
        "y": "0.000",
        "orientation": "0.000"
      },
      {
        "id": {
          "id": 4,
          "team": 2
        },
        "x": "-2.500",
        "y": "-0.000",
        "orientation": "0.000"
      },
      {
        "id": {
          "id": 5,
          "team": 2
        },
        "x": "-3.600",
        "y": "-0.000",
        "orientation": "0.000"
      },
      {
        "id": {
          "id": 6,
          "team": 2
        },
        "x": "-3.200",
        "y": "0.750",
        "orientation": "0.000"
      },
      {
        "id": {
          "id": 7,
          "team": 2
        },
        "x": "-3.200",
        "y": "-0.750",
        "orientation": "-0.001"
      },
      {
        "id": {
          "id": 8,
          "team": 2
        },
        "x": "-3.200",
        "y": "1.500",
        "orientation": "0.000"
      },
      {
        "id": {
          "id": 9,
          "team": 2
        },
        "x": "-3.198",
        "y": "-1.500",
        "orientation": "-0.012"
      },
      {
        "id": {
          "id": 10,
          "team": 2
        },
        "x": "-3.200",
        "y": "2.250",
        "orientation": "0.000"
      },
      {
        "id": {
          "id": 0,
          "team": 1
        },
        "x": "1.500",
        "y": "1.120",
        "orientation": "3.142"
      },
      {
        "id": {
          "id": 1,
          "team": 1
        },
        "x": "1.500",
        "y": "0.000",
        "orientation": "-3.142"
      },
      {
        "id": {
          "id": 2,
          "team": 1
        },
        "x": "1.500",
        "y": "-1.120",
        "orientation": "3.142"
      },
      {
        "id": {
          "id": 3,
          "team": 1
        },
        "x": "0.550",
        "y": "-0.000",
        "orientation": "-3.142"
      },
      {
        "id": {
          "id": 4,
          "team": 1
        },
        "x": "2.500",
        "y": "0.000",
        "orientation": "3.142"
      },
      {
        "id": {
          "id": 5,
          "team": 1
        },
        "x": "3.600",
        "y": "0.000",
        "orientation": "3.142"
      },
      {
        "id": {
          "id": 6,
          "team": 1
        },
        "x": "3.200",
        "y": "0.750",
        "orientation": "3.142"
      },
      {
        "id": {
          "id": 7,
          "team": 1
        },
        "x": "3.200",
        "y": "-0.750",
        "orientation": "3.142"
      },
      {
        "id": {
          "id": 8,
          "team": 1
        },
        "x": "3.200",
        "y": "1.500",
        "orientation": "3.142"
      },
      {
        "id": {
          "id": 9,
          "team": 1
        },
        "x": "3.200",
        "y": "-1.500",
        "orientation": "3.142"
      },
      {
        "id": {
          "id": 10,
          "team": 1
        },
        "x": "3.200",
        "y": "2.250",
        "orientation": "3.142"
      }
    ]
  }
};

export const FORMATION_2 = {
  "control": {
    "teleportBall": {
      "x": "0.000",
      "y": "0.000",
      "z": "0.021",
      "vx": "0.000",
      "vy": "0.000",
      "vz": "0.000"
    },
    "teleportRobot": [
      {
        "id": {
          "id": 0,
          "team": 2
        },
        "x": "-4.200",
        "y": "-0.000",
        "orientation": "0.000"
      },
      {
        "id": {
          "id": 1,
          "team": 2
        },
        "x": "-3.400",
        "y": "-0.200",
        "orientation": "0.000"
      },
      {
        "id": {
          "id": 2,
          "team": 2
        },
        "x": "-3.400",
        "y": "0.200",
        "orientation": "0.000"
      },
      {
        "id": {
          "id": 3,
          "team": 2
        },
        "x": "-0.700",
        "y": "-0.000",
        "orientation": "0.000"
      },
      {
        "id": {
          "id": 4,
          "team": 2
        },
        "x": "-0.700",
        "y": "2.250",
        "orientation": "0.000"
      },
      {
        "id": {
          "id": 5,
          "team": 2
        },
        "x": "-0.700",
        "y": "-2.250",
        "orientation": "0.000"
      },
      {
        "id": {
          "id": 6,
          "team": 2
        },
        "x": "-2.000",
        "y": "0.750",
        "orientation": "0.000"
      },
      {
        "id": {
          "id": 7,
          "team": 2
        },
        "x": "-2.000",
        "y": "-0.750",
        "orientation": "0.000"
      },
      {
        "id": {
          "id": 8,
          "team": 2
        },
        "x": "-2.000",
        "y": "1.500",
        "orientation": "0.000"
      },
      {
        "id": {
          "id": 9,
          "team": 2
        },
        "x": "-2.000",
        "y": "-1.500",
        "orientation": "0.000"
      },
      {
        "id": {
          "id": 10,
          "team": 2
        },
        "x": "-2.000",
        "y": "2.250",
        "orientation": "0.000"
      },
      {
        "id": {
          "id": 0,
          "team": 1
        },
        "x": "4.200",
        "y": "-0.000",
        "orientation": "-3.141"
      },
      {
        "id": {
          "id": 1,
          "team": 1
        },
        "x": "3.400",
        "y": "-0.200",
        "orientation": "3.141"
      },
      {
        "id": {
          "id": 2,
          "team": 1
        },
        "x": "3.400",
        "y": "0.200",
        "orientation": "3.141"
      },
      {
        "id": {
          "id": 3,
          "team": 1
        },
        "x": "0.700",
        "y": "0.000",
        "orientation": "3.141"
      },
      {
        "id": {
          "id": 4,
          "team": 1
        },
        "x": "0.700",
        "y": "2.250",
        "orientation": "-3.141"
      },
      {
        "id": {
          "id": 5,
          "team": 1
        },
        "x": "0.700",
        "y": "-2.250",
        "orientation": "-3.141"
      },
      {
        "id": {
          "id": 6,
          "team": 1
        },
        "x": "2.000",
        "y": "0.750",
        "orientation": "-3.141"
      },
      {
        "id": {
          "id": 7,
          "team": 1
        },
        "x": "2.000",
        "y": "-0.750",
        "orientation": "-3.141"
      },
      {
        "id": {
          "id": 8,
          "team": 1
        },
        "x": "2.000",
        "y": "1.500",
        "orientation": "-3.141"
      },
      {
        "id": {
          "id": 9,
          "team": 1
        },
        "x": "2.000",
        "y": "-1.500",
        "orientation": "-3.141"
      },
      {
        "id": {
          "id": 10,
          "team": 1
        },
        "x": "2.000",
        "y": "2.250",
        "orientation": "-3.141"
      }
    ]
  }
};

export const robotNameMap = (team: 'BLACK' | 'PURPLE', id: number) => {
  if (team === 'PURPLE') {
    return {
      1: 'Wall-E',
      2: 'R2D2',
      3: 'Tron',
      4: 'Marvin',
      5: 'Jarvis',
      6: 'Baymax',
      7: 'Noo-Noo',
      8: 'T-800',
      9: 'K-9',
      10: 'Bender',
      11: 'Holt',
      12: 'Chappie',
      13: 'TARS',
      14: '',
      15: 'Herman'
    }[id]
  }

  return ''
}

export const formatFloat = (pos?: number | null): string => {
  if (pos === undefined || pos === null) {
    return '0.00'
  }

  const value = Math.abs(pos).toFixed(2)
  const sign = pos > -0.0001 ? '+' : '-'
  return `${sign}${value}`
}
