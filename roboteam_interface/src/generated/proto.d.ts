import * as $protobuf from "protobufjs";
import Long = require("long");
/** Namespace proto. */
export namespace proto {

    /** Properties of a GameSettings. */
    interface IGameSettings {

        /** GameSettings isPrimaryAi */
        isPrimaryAi?: (boolean|null);

        /** GameSettings isYellow */
        isYellow?: (boolean|null);

        /** GameSettings isLeft */
        isLeft?: (boolean|null);

        /** GameSettings robotHubMode */
        robotHubMode?: (proto.GameSettings.RobotHubMode|null);
    }

    /** Represents a GameSettings. */
    class GameSettings implements IGameSettings {

        /**
         * Constructs a new GameSettings.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IGameSettings);

        /** GameSettings isPrimaryAi. */
        public isPrimaryAi: boolean;

        /** GameSettings isYellow. */
        public isYellow: boolean;

        /** GameSettings isLeft. */
        public isLeft: boolean;

        /** GameSettings robotHubMode. */
        public robotHubMode: proto.GameSettings.RobotHubMode;

        /**
         * Creates a new GameSettings instance using the specified properties.
         * @param [properties] Properties to set
         * @returns GameSettings instance
         */
        public static create(properties?: proto.IGameSettings): proto.GameSettings;

        /**
         * Encodes the specified GameSettings message. Does not implicitly {@link proto.GameSettings.verify|verify} messages.
         * @param message GameSettings message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IGameSettings, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified GameSettings message, length delimited. Does not implicitly {@link proto.GameSettings.verify|verify} messages.
         * @param message GameSettings message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IGameSettings, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a GameSettings message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns GameSettings
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameSettings;

        /**
         * Decodes a GameSettings message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns GameSettings
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameSettings;

        /**
         * Verifies a GameSettings message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a GameSettings message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns GameSettings
         */
        public static fromObject(object: { [k: string]: any }): proto.GameSettings;

        /**
         * Creates a plain object from a GameSettings message. Also converts values to other types if specified.
         * @param message GameSettings
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.GameSettings, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this GameSettings to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for GameSettings
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    namespace GameSettings {

        /** RobotHubMode enum. */
        enum RobotHubMode {
            UNKNOWN = 0,
            BASESTATION = 1,
            SIMULATOR = 2
        }
    }

    /** Properties of a Drawing. */
    interface IDrawing {

        /** Drawing retainForTicks */
        retainForTicks?: (number|null);

        /** Drawing label */
        label?: (string|null);

        /** Drawing color */
        color?: (proto.Drawing.Color|null);

        /** Drawing method */
        method?: (proto.Drawing.Method|null);

        /** Drawing points */
        points?: (proto.IVector2f[]|null);

        /** Drawing category */
        category?: (proto.Drawing.Category|null);

        /** Drawing forRobotId */
        forRobotId?: (number|null);

        /** Drawing size */
        size?: (number|null);

        /** Drawing thickness */
        thickness?: (number|null);
    }

    /** Represents a Drawing. */
    class Drawing implements IDrawing {

        /**
         * Constructs a new Drawing.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IDrawing);

        /** Drawing retainForTicks. */
        public retainForTicks: number;

        /** Drawing label. */
        public label: string;

        /** Drawing color. */
        public color: proto.Drawing.Color;

        /** Drawing method. */
        public method: proto.Drawing.Method;

        /** Drawing points. */
        public points: proto.IVector2f[];

        /** Drawing category. */
        public category: proto.Drawing.Category;

        /** Drawing forRobotId. */
        public forRobotId: number;

        /** Drawing size. */
        public size: number;

        /** Drawing thickness. */
        public thickness: number;

        /**
         * Creates a new Drawing instance using the specified properties.
         * @param [properties] Properties to set
         * @returns Drawing instance
         */
        public static create(properties?: proto.IDrawing): proto.Drawing;

        /**
         * Encodes the specified Drawing message. Does not implicitly {@link proto.Drawing.verify|verify} messages.
         * @param message Drawing message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IDrawing, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified Drawing message, length delimited. Does not implicitly {@link proto.Drawing.verify|verify} messages.
         * @param message Drawing message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IDrawing, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a Drawing message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns Drawing
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.Drawing;

        /**
         * Decodes a Drawing message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns Drawing
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.Drawing;

        /**
         * Verifies a Drawing message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a Drawing message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns Drawing
         */
        public static fromObject(object: { [k: string]: any }): proto.Drawing;

        /**
         * Creates a plain object from a Drawing message. Also converts values to other types if specified.
         * @param message Drawing
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.Drawing, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this Drawing to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for Drawing
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    namespace Drawing {

        /** Method enum. */
        enum Method {
            LINES_CONNECTED = 0,
            DOTS = 1,
            CROSSES = 2,
            PLUSES = 3,
            CIRCLES = 4
        }

        /** Color enum. */
        enum Color {
            RED = 0,
            GREEN = 1,
            BLUE = 2,
            YELLOW = 3,
            CYAN = 4,
            MAGENTA = 5,
            WHITE = 6,
            GREY = 7,
            BLACK = 8
        }

        /** Category enum. */
        enum Category {
            PATH_PLANNING = 0,
            DEBUG = 1,
            MARGINS = 2
        }
    }

    /** Properties of a Metric. */
    interface IMetric {

        /** Metric label */
        label?: (string|null);

        /** Metric boundedValue */
        boundedValue?: (proto.Metric.IBoundedValue|null);

        /** Metric decimal */
        decimal?: (proto.Metric.IDecimal|null);
    }

    /** Represents a Metric. */
    class Metric implements IMetric {

        /**
         * Constructs a new Metric.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IMetric);

        /** Metric label. */
        public label: string;

        /** Metric boundedValue. */
        public boundedValue?: (proto.Metric.IBoundedValue|null);

        /** Metric decimal. */
        public decimal?: (proto.Metric.IDecimal|null);

        /** Metric value. */
        public value?: ("boundedValue"|"decimal");

        /**
         * Creates a new Metric instance using the specified properties.
         * @param [properties] Properties to set
         * @returns Metric instance
         */
        public static create(properties?: proto.IMetric): proto.Metric;

        /**
         * Encodes the specified Metric message. Does not implicitly {@link proto.Metric.verify|verify} messages.
         * @param message Metric message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IMetric, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified Metric message, length delimited. Does not implicitly {@link proto.Metric.verify|verify} messages.
         * @param message Metric message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IMetric, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a Metric message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns Metric
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.Metric;

        /**
         * Decodes a Metric message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns Metric
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.Metric;

        /**
         * Verifies a Metric message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a Metric message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns Metric
         */
        public static fromObject(object: { [k: string]: any }): proto.Metric;

        /**
         * Creates a plain object from a Metric message. Also converts values to other types if specified.
         * @param message Metric
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.Metric, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this Metric to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for Metric
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    namespace Metric {

        /** Properties of a BoundedValue. */
        interface IBoundedValue {

            /** BoundedValue value */
            value?: (number|null);

            /** BoundedValue min */
            min?: (number|null);

            /** BoundedValue max */
            max?: (number|null);

            /** BoundedValue unit */
            unit?: (string|null);
        }

        /** Represents a BoundedValue. */
        class BoundedValue implements IBoundedValue {

            /**
             * Constructs a new BoundedValue.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.Metric.IBoundedValue);

            /** BoundedValue value. */
            public value: number;

            /** BoundedValue min. */
            public min: number;

            /** BoundedValue max. */
            public max: number;

            /** BoundedValue unit. */
            public unit: string;

            /**
             * Creates a new BoundedValue instance using the specified properties.
             * @param [properties] Properties to set
             * @returns BoundedValue instance
             */
            public static create(properties?: proto.Metric.IBoundedValue): proto.Metric.BoundedValue;

            /**
             * Encodes the specified BoundedValue message. Does not implicitly {@link proto.Metric.BoundedValue.verify|verify} messages.
             * @param message BoundedValue message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.Metric.IBoundedValue, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified BoundedValue message, length delimited. Does not implicitly {@link proto.Metric.BoundedValue.verify|verify} messages.
             * @param message BoundedValue message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.Metric.IBoundedValue, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a BoundedValue message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns BoundedValue
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.Metric.BoundedValue;

            /**
             * Decodes a BoundedValue message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns BoundedValue
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.Metric.BoundedValue;

            /**
             * Verifies a BoundedValue message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a BoundedValue message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns BoundedValue
             */
            public static fromObject(object: { [k: string]: any }): proto.Metric.BoundedValue;

            /**
             * Creates a plain object from a BoundedValue message. Also converts values to other types if specified.
             * @param message BoundedValue
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.Metric.BoundedValue, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this BoundedValue to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for BoundedValue
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a Decimal. */
        interface IDecimal {

            /** Decimal value */
            value?: (number|null);

            /** Decimal minRecorded */
            minRecorded?: (number|null);

            /** Decimal maxRecorded */
            maxRecorded?: (number|null);

            /** Decimal unit */
            unit?: (string|null);
        }

        /** Represents a Decimal. */
        class Decimal implements IDecimal {

            /**
             * Constructs a new Decimal.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.Metric.IDecimal);

            /** Decimal value. */
            public value: number;

            /** Decimal minRecorded. */
            public minRecorded: number;

            /** Decimal maxRecorded. */
            public maxRecorded: number;

            /** Decimal unit. */
            public unit: string;

            /**
             * Creates a new Decimal instance using the specified properties.
             * @param [properties] Properties to set
             * @returns Decimal instance
             */
            public static create(properties?: proto.Metric.IDecimal): proto.Metric.Decimal;

            /**
             * Encodes the specified Decimal message. Does not implicitly {@link proto.Metric.Decimal.verify|verify} messages.
             * @param message Decimal message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.Metric.IDecimal, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified Decimal message, length delimited. Does not implicitly {@link proto.Metric.Decimal.verify|verify} messages.
             * @param message Decimal message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.Metric.IDecimal, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a Decimal message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns Decimal
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.Metric.Decimal;

            /**
             * Decodes a Decimal message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns Decimal
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.Metric.Decimal;

            /**
             * Verifies a Decimal message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a Decimal message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns Decimal
             */
            public static fromObject(object: { [k: string]: any }): proto.Metric.Decimal;

            /**
             * Creates a plain object from a Decimal message. Also converts values to other types if specified.
             * @param message Decimal
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.Metric.Decimal, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this Decimal to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for Decimal
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }
    }

    /** Properties of a STPStatus. */
    interface ISTPStatus {

        /** STPStatus currentPlay */
        currentPlay?: (proto.IPlayInfo|null);

        /** STPStatus score */
        score?: (number|null);

        /** STPStatus robots */
        robots?: ({ [k: string]: proto.STPStatus.ISTPRobot }|null);

        /** STPStatus scoredPlays */
        scoredPlays?: (proto.STPStatus.IScoredPlay[]|null);

        /** STPStatus currentTick */
        currentTick?: (number|null);

        /** STPStatus tickDuration */
        tickDuration?: (number|null);

        /** STPStatus averageTickDuration */
        averageTickDuration?: (number|null);
    }

    /** Represents a STPStatus. */
    class STPStatus implements ISTPStatus {

        /**
         * Constructs a new STPStatus.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.ISTPStatus);

        /** STPStatus currentPlay. */
        public currentPlay?: (proto.IPlayInfo|null);

        /** STPStatus score. */
        public score: number;

        /** STPStatus robots. */
        public robots: { [k: string]: proto.STPStatus.ISTPRobot };

        /** STPStatus scoredPlays. */
        public scoredPlays: proto.STPStatus.IScoredPlay[];

        /** STPStatus currentTick. */
        public currentTick: number;

        /** STPStatus tickDuration. */
        public tickDuration: number;

        /** STPStatus averageTickDuration. */
        public averageTickDuration: number;

        /**
         * Creates a new STPStatus instance using the specified properties.
         * @param [properties] Properties to set
         * @returns STPStatus instance
         */
        public static create(properties?: proto.ISTPStatus): proto.STPStatus;

        /**
         * Encodes the specified STPStatus message. Does not implicitly {@link proto.STPStatus.verify|verify} messages.
         * @param message STPStatus message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.ISTPStatus, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified STPStatus message, length delimited. Does not implicitly {@link proto.STPStatus.verify|verify} messages.
         * @param message STPStatus message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.ISTPStatus, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a STPStatus message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns STPStatus
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.STPStatus;

        /**
         * Decodes a STPStatus message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns STPStatus
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.STPStatus;

        /**
         * Verifies a STPStatus message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a STPStatus message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns STPStatus
         */
        public static fromObject(object: { [k: string]: any }): proto.STPStatus;

        /**
         * Creates a plain object from a STPStatus message. Also converts values to other types if specified.
         * @param message STPStatus
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.STPStatus, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this STPStatus to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for STPStatus
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    namespace STPStatus {

        /** Properties of a STPRobot. */
        interface ISTPRobot {

            /** STPRobot id */
            id?: (number|null);

            /** STPRobot role */
            role?: (proto.STPStatus.STPRobot.IRole|null);

            /** STPRobot tactic */
            tactic?: (proto.STPStatus.STPRobot.ITactic|null);

            /** STPRobot skill */
            skill?: (proto.STPStatus.STPRobot.ISkill|null);
        }

        /** Represents a STPRobot. */
        class STPRobot implements ISTPRobot {

            /**
             * Constructs a new STPRobot.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.STPStatus.ISTPRobot);

            /** STPRobot id. */
            public id: number;

            /** STPRobot role. */
            public role?: (proto.STPStatus.STPRobot.IRole|null);

            /** STPRobot tactic. */
            public tactic?: (proto.STPStatus.STPRobot.ITactic|null);

            /** STPRobot skill. */
            public skill?: (proto.STPStatus.STPRobot.ISkill|null);

            /**
             * Creates a new STPRobot instance using the specified properties.
             * @param [properties] Properties to set
             * @returns STPRobot instance
             */
            public static create(properties?: proto.STPStatus.ISTPRobot): proto.STPStatus.STPRobot;

            /**
             * Encodes the specified STPRobot message. Does not implicitly {@link proto.STPStatus.STPRobot.verify|verify} messages.
             * @param message STPRobot message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.STPStatus.ISTPRobot, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified STPRobot message, length delimited. Does not implicitly {@link proto.STPStatus.STPRobot.verify|verify} messages.
             * @param message STPRobot message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.STPStatus.ISTPRobot, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a STPRobot message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns STPRobot
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.STPStatus.STPRobot;

            /**
             * Decodes a STPRobot message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns STPRobot
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.STPStatus.STPRobot;

            /**
             * Verifies a STPRobot message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a STPRobot message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns STPRobot
             */
            public static fromObject(object: { [k: string]: any }): proto.STPStatus.STPRobot;

            /**
             * Creates a plain object from a STPRobot message. Also converts values to other types if specified.
             * @param message STPRobot
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.STPStatus.STPRobot, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this STPRobot to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for STPRobot
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        namespace STPRobot {

            /** Status enum. */
            enum Status {
                WAITING = 0,
                SUCCESSFUL = 1,
                FAILURE = 2,
                RUNNING = 3
            }

            /** Properties of a Role. */
            interface IRole {

                /** Role name */
                name?: (string|null);

                /** Role status */
                status?: (proto.STPStatus.STPRobot.Status|null);
            }

            /** Represents a Role. */
            class Role implements IRole {

                /**
                 * Constructs a new Role.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: proto.STPStatus.STPRobot.IRole);

                /** Role name. */
                public name: string;

                /** Role status. */
                public status: proto.STPStatus.STPRobot.Status;

                /**
                 * Creates a new Role instance using the specified properties.
                 * @param [properties] Properties to set
                 * @returns Role instance
                 */
                public static create(properties?: proto.STPStatus.STPRobot.IRole): proto.STPStatus.STPRobot.Role;

                /**
                 * Encodes the specified Role message. Does not implicitly {@link proto.STPStatus.STPRobot.Role.verify|verify} messages.
                 * @param message Role message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: proto.STPStatus.STPRobot.IRole, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Encodes the specified Role message, length delimited. Does not implicitly {@link proto.STPStatus.STPRobot.Role.verify|verify} messages.
                 * @param message Role message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encodeDelimited(message: proto.STPStatus.STPRobot.IRole, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a Role message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns Role
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.STPStatus.STPRobot.Role;

                /**
                 * Decodes a Role message from the specified reader or buffer, length delimited.
                 * @param reader Reader or buffer to decode from
                 * @returns Role
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.STPStatus.STPRobot.Role;

                /**
                 * Verifies a Role message.
                 * @param message Plain object to verify
                 * @returns `null` if valid, otherwise the reason why it is not
                 */
                public static verify(message: { [k: string]: any }): (string|null);

                /**
                 * Creates a Role message from a plain object. Also converts values to their respective internal types.
                 * @param object Plain object
                 * @returns Role
                 */
                public static fromObject(object: { [k: string]: any }): proto.STPStatus.STPRobot.Role;

                /**
                 * Creates a plain object from a Role message. Also converts values to other types if specified.
                 * @param message Role
                 * @param [options] Conversion options
                 * @returns Plain object
                 */
                public static toObject(message: proto.STPStatus.STPRobot.Role, options?: $protobuf.IConversionOptions): { [k: string]: any };

                /**
                 * Converts this Role to JSON.
                 * @returns JSON object
                 */
                public toJSON(): { [k: string]: any };

                /**
                 * Gets the default type url for Role
                 * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
                 * @returns The default type url
                 */
                public static getTypeUrl(typeUrlPrefix?: string): string;
            }

            /** Properties of a Tactic. */
            interface ITactic {

                /** Tactic name */
                name?: (string|null);

                /** Tactic status */
                status?: (proto.STPStatus.STPRobot.Status|null);
            }

            /** Represents a Tactic. */
            class Tactic implements ITactic {

                /**
                 * Constructs a new Tactic.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: proto.STPStatus.STPRobot.ITactic);

                /** Tactic name. */
                public name: string;

                /** Tactic status. */
                public status: proto.STPStatus.STPRobot.Status;

                /**
                 * Creates a new Tactic instance using the specified properties.
                 * @param [properties] Properties to set
                 * @returns Tactic instance
                 */
                public static create(properties?: proto.STPStatus.STPRobot.ITactic): proto.STPStatus.STPRobot.Tactic;

                /**
                 * Encodes the specified Tactic message. Does not implicitly {@link proto.STPStatus.STPRobot.Tactic.verify|verify} messages.
                 * @param message Tactic message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: proto.STPStatus.STPRobot.ITactic, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Encodes the specified Tactic message, length delimited. Does not implicitly {@link proto.STPStatus.STPRobot.Tactic.verify|verify} messages.
                 * @param message Tactic message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encodeDelimited(message: proto.STPStatus.STPRobot.ITactic, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a Tactic message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns Tactic
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.STPStatus.STPRobot.Tactic;

                /**
                 * Decodes a Tactic message from the specified reader or buffer, length delimited.
                 * @param reader Reader or buffer to decode from
                 * @returns Tactic
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.STPStatus.STPRobot.Tactic;

                /**
                 * Verifies a Tactic message.
                 * @param message Plain object to verify
                 * @returns `null` if valid, otherwise the reason why it is not
                 */
                public static verify(message: { [k: string]: any }): (string|null);

                /**
                 * Creates a Tactic message from a plain object. Also converts values to their respective internal types.
                 * @param object Plain object
                 * @returns Tactic
                 */
                public static fromObject(object: { [k: string]: any }): proto.STPStatus.STPRobot.Tactic;

                /**
                 * Creates a plain object from a Tactic message. Also converts values to other types if specified.
                 * @param message Tactic
                 * @param [options] Conversion options
                 * @returns Plain object
                 */
                public static toObject(message: proto.STPStatus.STPRobot.Tactic, options?: $protobuf.IConversionOptions): { [k: string]: any };

                /**
                 * Converts this Tactic to JSON.
                 * @returns JSON object
                 */
                public toJSON(): { [k: string]: any };

                /**
                 * Gets the default type url for Tactic
                 * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
                 * @returns The default type url
                 */
                public static getTypeUrl(typeUrlPrefix?: string): string;
            }

            /** Properties of a Skill. */
            interface ISkill {

                /** Skill name */
                name?: (string|null);

                /** Skill status */
                status?: (proto.STPStatus.STPRobot.Status|null);
            }

            /** Represents a Skill. */
            class Skill implements ISkill {

                /**
                 * Constructs a new Skill.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: proto.STPStatus.STPRobot.ISkill);

                /** Skill name. */
                public name: string;

                /** Skill status. */
                public status: proto.STPStatus.STPRobot.Status;

                /**
                 * Creates a new Skill instance using the specified properties.
                 * @param [properties] Properties to set
                 * @returns Skill instance
                 */
                public static create(properties?: proto.STPStatus.STPRobot.ISkill): proto.STPStatus.STPRobot.Skill;

                /**
                 * Encodes the specified Skill message. Does not implicitly {@link proto.STPStatus.STPRobot.Skill.verify|verify} messages.
                 * @param message Skill message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: proto.STPStatus.STPRobot.ISkill, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Encodes the specified Skill message, length delimited. Does not implicitly {@link proto.STPStatus.STPRobot.Skill.verify|verify} messages.
                 * @param message Skill message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encodeDelimited(message: proto.STPStatus.STPRobot.ISkill, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a Skill message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns Skill
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.STPStatus.STPRobot.Skill;

                /**
                 * Decodes a Skill message from the specified reader or buffer, length delimited.
                 * @param reader Reader or buffer to decode from
                 * @returns Skill
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.STPStatus.STPRobot.Skill;

                /**
                 * Verifies a Skill message.
                 * @param message Plain object to verify
                 * @returns `null` if valid, otherwise the reason why it is not
                 */
                public static verify(message: { [k: string]: any }): (string|null);

                /**
                 * Creates a Skill message from a plain object. Also converts values to their respective internal types.
                 * @param object Plain object
                 * @returns Skill
                 */
                public static fromObject(object: { [k: string]: any }): proto.STPStatus.STPRobot.Skill;

                /**
                 * Creates a plain object from a Skill message. Also converts values to other types if specified.
                 * @param message Skill
                 * @param [options] Conversion options
                 * @returns Plain object
                 */
                public static toObject(message: proto.STPStatus.STPRobot.Skill, options?: $protobuf.IConversionOptions): { [k: string]: any };

                /**
                 * Converts this Skill to JSON.
                 * @returns JSON object
                 */
                public toJSON(): { [k: string]: any };

                /**
                 * Gets the default type url for Skill
                 * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
                 * @returns The default type url
                 */
                public static getTypeUrl(typeUrlPrefix?: string): string;
            }
        }

        /** Properties of a ScoredPlay. */
        interface IScoredPlay {

            /** ScoredPlay playName */
            playName?: (string|null);

            /** ScoredPlay playScore */
            playScore?: (number|null);
        }

        /** Represents a ScoredPlay. */
        class ScoredPlay implements IScoredPlay {

            /**
             * Constructs a new ScoredPlay.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.STPStatus.IScoredPlay);

            /** ScoredPlay playName. */
            public playName: string;

            /** ScoredPlay playScore. */
            public playScore: number;

            /**
             * Creates a new ScoredPlay instance using the specified properties.
             * @param [properties] Properties to set
             * @returns ScoredPlay instance
             */
            public static create(properties?: proto.STPStatus.IScoredPlay): proto.STPStatus.ScoredPlay;

            /**
             * Encodes the specified ScoredPlay message. Does not implicitly {@link proto.STPStatus.ScoredPlay.verify|verify} messages.
             * @param message ScoredPlay message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.STPStatus.IScoredPlay, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified ScoredPlay message, length delimited. Does not implicitly {@link proto.STPStatus.ScoredPlay.verify|verify} messages.
             * @param message ScoredPlay message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.STPStatus.IScoredPlay, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a ScoredPlay message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns ScoredPlay
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.STPStatus.ScoredPlay;

            /**
             * Decodes a ScoredPlay message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns ScoredPlay
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.STPStatus.ScoredPlay;

            /**
             * Verifies a ScoredPlay message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a ScoredPlay message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns ScoredPlay
             */
            public static fromObject(object: { [k: string]: any }): proto.STPStatus.ScoredPlay;

            /**
             * Creates a plain object from a ScoredPlay message. Also converts values to other types if specified.
             * @param message ScoredPlay
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.STPStatus.ScoredPlay, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this ScoredPlay to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for ScoredPlay
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }
    }

    /** Properties of a RuntimeConfig. */
    interface IRuntimeConfig {

        /** RuntimeConfig useReferee */
        useReferee?: (boolean|null);

        /** RuntimeConfig ignoreInvariants */
        ignoreInvariants?: (boolean|null);
    }

    /** Represents a RuntimeConfig. */
    class RuntimeConfig implements IRuntimeConfig {

        /**
         * Constructs a new RuntimeConfig.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IRuntimeConfig);

        /** RuntimeConfig useReferee. */
        public useReferee: boolean;

        /** RuntimeConfig ignoreInvariants. */
        public ignoreInvariants: boolean;

        /**
         * Creates a new RuntimeConfig instance using the specified properties.
         * @param [properties] Properties to set
         * @returns RuntimeConfig instance
         */
        public static create(properties?: proto.IRuntimeConfig): proto.RuntimeConfig;

        /**
         * Encodes the specified RuntimeConfig message. Does not implicitly {@link proto.RuntimeConfig.verify|verify} messages.
         * @param message RuntimeConfig message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IRuntimeConfig, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified RuntimeConfig message, length delimited. Does not implicitly {@link proto.RuntimeConfig.verify|verify} messages.
         * @param message RuntimeConfig message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IRuntimeConfig, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a RuntimeConfig message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns RuntimeConfig
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.RuntimeConfig;

        /**
         * Decodes a RuntimeConfig message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns RuntimeConfig
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.RuntimeConfig;

        /**
         * Verifies a RuntimeConfig message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a RuntimeConfig message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns RuntimeConfig
         */
        public static fromObject(object: { [k: string]: any }): proto.RuntimeConfig;

        /**
         * Creates a plain object from a RuntimeConfig message. Also converts values to other types if specified.
         * @param message RuntimeConfig
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.RuntimeConfig, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this RuntimeConfig to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for RuntimeConfig
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a AIState. */
    interface IAIState {

        /** AIState isPaused */
        isPaused?: (boolean|null);

        /** AIState plays */
        plays?: (string[]|null);

        /** AIState ruleSets */
        ruleSets?: (string[]|null);

        /** AIState gameSettings */
        gameSettings?: (proto.IGameSettings|null);

        /** AIState runtimeConfig */
        runtimeConfig?: (proto.IRuntimeConfig|null);
    }

    /** Represents a AIState. */
    class AIState implements IAIState {

        /**
         * Constructs a new AIState.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IAIState);

        /** AIState isPaused. */
        public isPaused: boolean;

        /** AIState plays. */
        public plays: string[];

        /** AIState ruleSets. */
        public ruleSets: string[];

        /** AIState gameSettings. */
        public gameSettings?: (proto.IGameSettings|null);

        /** AIState runtimeConfig. */
        public runtimeConfig?: (proto.IRuntimeConfig|null);

        /**
         * Creates a new AIState instance using the specified properties.
         * @param [properties] Properties to set
         * @returns AIState instance
         */
        public static create(properties?: proto.IAIState): proto.AIState;

        /**
         * Encodes the specified AIState message. Does not implicitly {@link proto.AIState.verify|verify} messages.
         * @param message AIState message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IAIState, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified AIState message, length delimited. Does not implicitly {@link proto.AIState.verify|verify} messages.
         * @param message AIState message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IAIState, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a AIState message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns AIState
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.AIState;

        /**
         * Decodes a AIState message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns AIState
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.AIState;

        /**
         * Verifies a AIState message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a AIState message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns AIState
         */
        public static fromObject(object: { [k: string]: any }): proto.AIState;

        /**
         * Creates a plain object from a AIState message. Also converts values to other types if specified.
         * @param message AIState
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.AIState, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this AIState to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for AIState
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a PlayInfo. */
    interface IPlayInfo {

        /** PlayInfo playName */
        playName?: (string|null);

        /** PlayInfo rulesetName */
        rulesetName?: (string|null);

        /** PlayInfo keeperId */
        keeperId?: (number|null);

        /** PlayInfo timeleft */
        timeleft?: (number|null);

        /** PlayInfo followupcommandfromrefName */
        followupcommandfromrefName?: (string|null);

        /** PlayInfo commandfromrefName */
        commandfromrefName?: (string|null);
    }

    /** Represents a PlayInfo. */
    class PlayInfo implements IPlayInfo {

        /**
         * Constructs a new PlayInfo.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IPlayInfo);

        /** PlayInfo playName. */
        public playName: string;

        /** PlayInfo rulesetName. */
        public rulesetName: string;

        /** PlayInfo keeperId. */
        public keeperId: number;

        /** PlayInfo timeleft. */
        public timeleft: number;

        /** PlayInfo followupcommandfromrefName. */
        public followupcommandfromrefName: string;

        /** PlayInfo commandfromrefName. */
        public commandfromrefName: string;

        /**
         * Creates a new PlayInfo instance using the specified properties.
         * @param [properties] Properties to set
         * @returns PlayInfo instance
         */
        public static create(properties?: proto.IPlayInfo): proto.PlayInfo;

        /**
         * Encodes the specified PlayInfo message. Does not implicitly {@link proto.PlayInfo.verify|verify} messages.
         * @param message PlayInfo message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IPlayInfo, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified PlayInfo message, length delimited. Does not implicitly {@link proto.PlayInfo.verify|verify} messages.
         * @param message PlayInfo message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IPlayInfo, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a PlayInfo message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns PlayInfo
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.PlayInfo;

        /**
         * Decodes a PlayInfo message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns PlayInfo
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.PlayInfo;

        /**
         * Verifies a PlayInfo message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a PlayInfo message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns PlayInfo
         */
        public static fromObject(object: { [k: string]: any }): proto.PlayInfo;

        /**
         * Creates a plain object from a PlayInfo message. Also converts values to other types if specified.
         * @param message PlayInfo
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.PlayInfo, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this PlayInfo to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for PlayInfo
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a MsgToInterface. */
    interface IMsgToInterface {

        /** MsgToInterface stpStatus */
        stpStatus?: (proto.ISTPStatus|null);

        /** MsgToInterface aiState */
        aiState?: (proto.IAIState|null);

        /** MsgToInterface state */
        state?: (proto.IState|null);

        /** MsgToInterface visualizations */
        visualizations?: (proto.MsgToInterface.IVisualizationBuffer|null);
    }

    /** Represents a MsgToInterface. */
    class MsgToInterface implements IMsgToInterface {

        /**
         * Constructs a new MsgToInterface.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IMsgToInterface);

        /** MsgToInterface stpStatus. */
        public stpStatus?: (proto.ISTPStatus|null);

        /** MsgToInterface aiState. */
        public aiState?: (proto.IAIState|null);

        /** MsgToInterface state. */
        public state?: (proto.IState|null);

        /** MsgToInterface visualizations. */
        public visualizations?: (proto.MsgToInterface.IVisualizationBuffer|null);

        /** MsgToInterface kind. */
        public kind?: ("stpStatus"|"aiState"|"state"|"visualizations");

        /**
         * Creates a new MsgToInterface instance using the specified properties.
         * @param [properties] Properties to set
         * @returns MsgToInterface instance
         */
        public static create(properties?: proto.IMsgToInterface): proto.MsgToInterface;

        /**
         * Encodes the specified MsgToInterface message. Does not implicitly {@link proto.MsgToInterface.verify|verify} messages.
         * @param message MsgToInterface message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IMsgToInterface, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified MsgToInterface message, length delimited. Does not implicitly {@link proto.MsgToInterface.verify|verify} messages.
         * @param message MsgToInterface message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IMsgToInterface, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a MsgToInterface message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns MsgToInterface
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.MsgToInterface;

        /**
         * Decodes a MsgToInterface message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns MsgToInterface
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.MsgToInterface;

        /**
         * Verifies a MsgToInterface message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a MsgToInterface message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns MsgToInterface
         */
        public static fromObject(object: { [k: string]: any }): proto.MsgToInterface;

        /**
         * Creates a plain object from a MsgToInterface message. Also converts values to other types if specified.
         * @param message MsgToInterface
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.MsgToInterface, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this MsgToInterface to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for MsgToInterface
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    namespace MsgToInterface {

        /** Properties of a VisualizationBuffer. */
        interface IVisualizationBuffer {

            /** VisualizationBuffer drawings */
            drawings?: (proto.IDrawing[]|null);

            /** VisualizationBuffer metrics */
            metrics?: (proto.IMetric[]|null);
        }

        /** Represents a VisualizationBuffer. */
        class VisualizationBuffer implements IVisualizationBuffer {

            /**
             * Constructs a new VisualizationBuffer.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.MsgToInterface.IVisualizationBuffer);

            /** VisualizationBuffer drawings. */
            public drawings: proto.IDrawing[];

            /** VisualizationBuffer metrics. */
            public metrics: proto.IMetric[];

            /**
             * Creates a new VisualizationBuffer instance using the specified properties.
             * @param [properties] Properties to set
             * @returns VisualizationBuffer instance
             */
            public static create(properties?: proto.MsgToInterface.IVisualizationBuffer): proto.MsgToInterface.VisualizationBuffer;

            /**
             * Encodes the specified VisualizationBuffer message. Does not implicitly {@link proto.MsgToInterface.VisualizationBuffer.verify|verify} messages.
             * @param message VisualizationBuffer message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.MsgToInterface.IVisualizationBuffer, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified VisualizationBuffer message, length delimited. Does not implicitly {@link proto.MsgToInterface.VisualizationBuffer.verify|verify} messages.
             * @param message VisualizationBuffer message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.MsgToInterface.IVisualizationBuffer, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a VisualizationBuffer message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns VisualizationBuffer
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.MsgToInterface.VisualizationBuffer;

            /**
             * Decodes a VisualizationBuffer message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns VisualizationBuffer
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.MsgToInterface.VisualizationBuffer;

            /**
             * Verifies a VisualizationBuffer message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a VisualizationBuffer message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns VisualizationBuffer
             */
            public static fromObject(object: { [k: string]: any }): proto.MsgToInterface.VisualizationBuffer;

            /**
             * Creates a plain object from a VisualizationBuffer message. Also converts values to other types if specified.
             * @param message VisualizationBuffer
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.MsgToInterface.VisualizationBuffer, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this VisualizationBuffer to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for VisualizationBuffer
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }
    }

    /** Properties of a MsgFromInterface. */
    interface IMsgFromInterface {

        /** MsgFromInterface setPlay */
        setPlay?: (proto.IPlayInfo|null);

        /** MsgFromInterface setGameSettings */
        setGameSettings?: (proto.IGameSettings|null);

        /** MsgFromInterface setRuntimeConfig */
        setRuntimeConfig?: (proto.IRuntimeConfig|null);

        /** MsgFromInterface pauseAi */
        pauseAi?: (boolean|null);

        /** MsgFromInterface setBallPos */
        setBallPos?: (proto.IVector2f|null);

        /** MsgFromInterface simulatorCommand */
        simulatorCommand?: (ISimulatorCommand|null);
    }

    /** Represents a MsgFromInterface. */
    class MsgFromInterface implements IMsgFromInterface {

        /**
         * Constructs a new MsgFromInterface.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IMsgFromInterface);

        /** MsgFromInterface setPlay. */
        public setPlay?: (proto.IPlayInfo|null);

        /** MsgFromInterface setGameSettings. */
        public setGameSettings?: (proto.IGameSettings|null);

        /** MsgFromInterface setRuntimeConfig. */
        public setRuntimeConfig?: (proto.IRuntimeConfig|null);

        /** MsgFromInterface pauseAi. */
        public pauseAi?: (boolean|null);

        /** MsgFromInterface setBallPos. */
        public setBallPos?: (proto.IVector2f|null);

        /** MsgFromInterface simulatorCommand. */
        public simulatorCommand?: (ISimulatorCommand|null);

        /** MsgFromInterface kind. */
        public kind?: ("setPlay"|"setGameSettings"|"setRuntimeConfig"|"pauseAi"|"setBallPos"|"simulatorCommand");

        /**
         * Creates a new MsgFromInterface instance using the specified properties.
         * @param [properties] Properties to set
         * @returns MsgFromInterface instance
         */
        public static create(properties?: proto.IMsgFromInterface): proto.MsgFromInterface;

        /**
         * Encodes the specified MsgFromInterface message. Does not implicitly {@link proto.MsgFromInterface.verify|verify} messages.
         * @param message MsgFromInterface message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IMsgFromInterface, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified MsgFromInterface message, length delimited. Does not implicitly {@link proto.MsgFromInterface.verify|verify} messages.
         * @param message MsgFromInterface message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IMsgFromInterface, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a MsgFromInterface message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns MsgFromInterface
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.MsgFromInterface;

        /**
         * Decodes a MsgFromInterface message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns MsgFromInterface
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.MsgFromInterface;

        /**
         * Verifies a MsgFromInterface message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a MsgFromInterface message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns MsgFromInterface
         */
        public static fromObject(object: { [k: string]: any }): proto.MsgFromInterface;

        /**
         * Creates a plain object from a MsgFromInterface message. Also converts values to other types if specified.
         * @param message MsgFromInterface
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.MsgFromInterface, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this MsgFromInterface to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for MsgFromInterface
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a State. */
    interface IState {

        /** State lastSeenWorld */
        lastSeenWorld?: (proto.IWorld|null);

        /** State commandExtrapolatedWorld */
        commandExtrapolatedWorld?: (proto.IWorld|null);

        /** State ballCameraWorld */
        ballCameraWorld?: (proto.IWorld|null);

        /** State blueRobotParameters */
        blueRobotParameters?: (proto.ITeamParameters|null);

        /** State yellowRobotParameters */
        yellowRobotParameters?: (proto.ITeamParameters|null);

        /** State field */
        field?: (proto.ISSL_GeometryData|null);

        /** State referee */
        referee?: (proto.IReferee|null);

        /** State processedVisionPackets */
        processedVisionPackets?: (proto.ISSL_WrapperPacket[]|null);

        /** State processedRefereePackets */
        processedRefereePackets?: (proto.IReferee[]|null);

        /** State processedFeedbackPackets */
        processedFeedbackPackets?: (proto.IRobotsFeedback[]|null);
    }

    /** Represents a State. */
    class State implements IState {

        /**
         * Constructs a new State.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IState);

        /** State lastSeenWorld. */
        public lastSeenWorld?: (proto.IWorld|null);

        /** State commandExtrapolatedWorld. */
        public commandExtrapolatedWorld?: (proto.IWorld|null);

        /** State ballCameraWorld. */
        public ballCameraWorld?: (proto.IWorld|null);

        /** State blueRobotParameters. */
        public blueRobotParameters?: (proto.ITeamParameters|null);

        /** State yellowRobotParameters. */
        public yellowRobotParameters?: (proto.ITeamParameters|null);

        /** State field. */
        public field?: (proto.ISSL_GeometryData|null);

        /** State referee. */
        public referee?: (proto.IReferee|null);

        /** State processedVisionPackets. */
        public processedVisionPackets: proto.ISSL_WrapperPacket[];

        /** State processedRefereePackets. */
        public processedRefereePackets: proto.IReferee[];

        /** State processedFeedbackPackets. */
        public processedFeedbackPackets: proto.IRobotsFeedback[];

        /**
         * Creates a new State instance using the specified properties.
         * @param [properties] Properties to set
         * @returns State instance
         */
        public static create(properties?: proto.IState): proto.State;

        /**
         * Encodes the specified State message. Does not implicitly {@link proto.State.verify|verify} messages.
         * @param message State message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IState, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified State message, length delimited. Does not implicitly {@link proto.State.verify|verify} messages.
         * @param message State message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IState, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a State message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns State
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.State;

        /**
         * Decodes a State message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns State
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.State;

        /**
         * Verifies a State message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a State message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns State
         */
        public static fromObject(object: { [k: string]: any }): proto.State;

        /**
         * Creates a plain object from a State message. Also converts values to other types if specified.
         * @param message State
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.State, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this State to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for State
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a World. */
    interface IWorld {

        /** World time */
        time?: (number|Long|null);

        /** World id */
        id?: (number|null);

        /** World ball */
        ball?: (proto.IWorldBall|null);

        /** World yellow */
        yellow?: (proto.IWorldRobot[]|null);

        /** World blue */
        blue?: (proto.IWorldRobot[]|null);

        /** World yellowUnseenRobots */
        yellowUnseenRobots?: (proto.IFeedbackOnlyRobot[]|null);

        /** World blueUnseenRobots */
        blueUnseenRobots?: (proto.IFeedbackOnlyRobot[]|null);
    }

    /** Represents a World. */
    class World implements IWorld {

        /**
         * Constructs a new World.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IWorld);

        /** World time. */
        public time: (number|Long);

        /** World id. */
        public id: number;

        /** World ball. */
        public ball?: (proto.IWorldBall|null);

        /** World yellow. */
        public yellow: proto.IWorldRobot[];

        /** World blue. */
        public blue: proto.IWorldRobot[];

        /** World yellowUnseenRobots. */
        public yellowUnseenRobots: proto.IFeedbackOnlyRobot[];

        /** World blueUnseenRobots. */
        public blueUnseenRobots: proto.IFeedbackOnlyRobot[];

        /**
         * Creates a new World instance using the specified properties.
         * @param [properties] Properties to set
         * @returns World instance
         */
        public static create(properties?: proto.IWorld): proto.World;

        /**
         * Encodes the specified World message. Does not implicitly {@link proto.World.verify|verify} messages.
         * @param message World message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IWorld, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified World message, length delimited. Does not implicitly {@link proto.World.verify|verify} messages.
         * @param message World message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IWorld, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a World message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns World
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.World;

        /**
         * Decodes a World message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns World
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.World;

        /**
         * Verifies a World message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a World message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns World
         */
        public static fromObject(object: { [k: string]: any }): proto.World;

        /**
         * Creates a plain object from a World message. Also converts values to other types if specified.
         * @param message World
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.World, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this World to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for World
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a WorldBall. */
    interface IWorldBall {

        /** WorldBall area */
        area?: (number|null);

        /** WorldBall pos */
        pos?: (proto.IVector2f|null);

        /** WorldBall z */
        z?: (number|null);

        /** WorldBall vel */
        vel?: (proto.IVector2f|null);

        /** WorldBall zVel */
        zVel?: (number|null);

        /** WorldBall visible */
        visible?: (boolean|null);
    }

    /** Represents a WorldBall. */
    class WorldBall implements IWorldBall {

        /**
         * Constructs a new WorldBall.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IWorldBall);

        /** WorldBall area. */
        public area: number;

        /** WorldBall pos. */
        public pos?: (proto.IVector2f|null);

        /** WorldBall z. */
        public z: number;

        /** WorldBall vel. */
        public vel?: (proto.IVector2f|null);

        /** WorldBall zVel. */
        public zVel: number;

        /** WorldBall visible. */
        public visible: boolean;

        /**
         * Creates a new WorldBall instance using the specified properties.
         * @param [properties] Properties to set
         * @returns WorldBall instance
         */
        public static create(properties?: proto.IWorldBall): proto.WorldBall;

        /**
         * Encodes the specified WorldBall message. Does not implicitly {@link proto.WorldBall.verify|verify} messages.
         * @param message WorldBall message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IWorldBall, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified WorldBall message, length delimited. Does not implicitly {@link proto.WorldBall.verify|verify} messages.
         * @param message WorldBall message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IWorldBall, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a WorldBall message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns WorldBall
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.WorldBall;

        /**
         * Decodes a WorldBall message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns WorldBall
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.WorldBall;

        /**
         * Verifies a WorldBall message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a WorldBall message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns WorldBall
         */
        public static fromObject(object: { [k: string]: any }): proto.WorldBall;

        /**
         * Creates a plain object from a WorldBall message. Also converts values to other types if specified.
         * @param message WorldBall
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.WorldBall, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this WorldBall to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for WorldBall
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a Vector2f. */
    interface IVector2f {

        /** Vector2f x */
        x?: (number|null);

        /** Vector2f y */
        y?: (number|null);
    }

    /** Represents a Vector2f. */
    class Vector2f implements IVector2f {

        /**
         * Constructs a new Vector2f.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IVector2f);

        /** Vector2f x. */
        public x: number;

        /** Vector2f y. */
        public y: number;

        /**
         * Creates a new Vector2f instance using the specified properties.
         * @param [properties] Properties to set
         * @returns Vector2f instance
         */
        public static create(properties?: proto.IVector2f): proto.Vector2f;

        /**
         * Encodes the specified Vector2f message. Does not implicitly {@link proto.Vector2f.verify|verify} messages.
         * @param message Vector2f message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IVector2f, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified Vector2f message, length delimited. Does not implicitly {@link proto.Vector2f.verify|verify} messages.
         * @param message Vector2f message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IVector2f, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a Vector2f message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns Vector2f
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.Vector2f;

        /**
         * Decodes a Vector2f message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns Vector2f
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.Vector2f;

        /**
         * Verifies a Vector2f message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a Vector2f message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns Vector2f
         */
        public static fromObject(object: { [k: string]: any }): proto.Vector2f;

        /**
         * Creates a plain object from a Vector2f message. Also converts values to other types if specified.
         * @param message Vector2f
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.Vector2f, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this Vector2f to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for Vector2f
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a WorldRobot. */
    interface IWorldRobot {

        /** WorldRobot id */
        id?: (number|null);

        /** WorldRobot pos */
        pos?: (proto.IVector2f|null);

        /** WorldRobot yaw */
        yaw?: (number|null);

        /** WorldRobot vel */
        vel?: (proto.IVector2f|null);

        /** WorldRobot w */
        w?: (number|null);

        /** WorldRobot feedbackInfo */
        feedbackInfo?: (proto.IRobotProcessedFeedback|null);
    }

    /** Represents a WorldRobot. */
    class WorldRobot implements IWorldRobot {

        /**
         * Constructs a new WorldRobot.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IWorldRobot);

        /** WorldRobot id. */
        public id: number;

        /** WorldRobot pos. */
        public pos?: (proto.IVector2f|null);

        /** WorldRobot yaw. */
        public yaw: number;

        /** WorldRobot vel. */
        public vel?: (proto.IVector2f|null);

        /** WorldRobot w. */
        public w: number;

        /** WorldRobot feedbackInfo. */
        public feedbackInfo?: (proto.IRobotProcessedFeedback|null);

        /**
         * Creates a new WorldRobot instance using the specified properties.
         * @param [properties] Properties to set
         * @returns WorldRobot instance
         */
        public static create(properties?: proto.IWorldRobot): proto.WorldRobot;

        /**
         * Encodes the specified WorldRobot message. Does not implicitly {@link proto.WorldRobot.verify|verify} messages.
         * @param message WorldRobot message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IWorldRobot, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified WorldRobot message, length delimited. Does not implicitly {@link proto.WorldRobot.verify|verify} messages.
         * @param message WorldRobot message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IWorldRobot, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a WorldRobot message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns WorldRobot
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.WorldRobot;

        /**
         * Decodes a WorldRobot message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns WorldRobot
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.WorldRobot;

        /**
         * Verifies a WorldRobot message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a WorldRobot message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns WorldRobot
         */
        public static fromObject(object: { [k: string]: any }): proto.WorldRobot;

        /**
         * Creates a plain object from a WorldRobot message. Also converts values to other types if specified.
         * @param message WorldRobot
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.WorldRobot, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this WorldRobot to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for WorldRobot
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a RobotProcessedFeedback. */
    interface IRobotProcessedFeedback {

        /** RobotProcessedFeedback ballSensorSeesBall */
        ballSensorSeesBall?: (boolean|null);

        /** RobotProcessedFeedback ballSensorIsWorking */
        ballSensorIsWorking?: (boolean|null);

        /** RobotProcessedFeedback dribblerSeesBall */
        dribblerSeesBall?: (boolean|null);

        /** RobotProcessedFeedback xsensIsCalibrated */
        xsensIsCalibrated?: (boolean|null);

        /** RobotProcessedFeedback capacitorIsCharged */
        capacitorIsCharged?: (boolean|null);

        /** RobotProcessedFeedback batteryLevel */
        batteryLevel?: (number|null);
    }

    /** Represents a RobotProcessedFeedback. */
    class RobotProcessedFeedback implements IRobotProcessedFeedback {

        /**
         * Constructs a new RobotProcessedFeedback.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IRobotProcessedFeedback);

        /** RobotProcessedFeedback ballSensorSeesBall. */
        public ballSensorSeesBall: boolean;

        /** RobotProcessedFeedback ballSensorIsWorking. */
        public ballSensorIsWorking: boolean;

        /** RobotProcessedFeedback dribblerSeesBall. */
        public dribblerSeesBall: boolean;

        /** RobotProcessedFeedback xsensIsCalibrated. */
        public xsensIsCalibrated: boolean;

        /** RobotProcessedFeedback capacitorIsCharged. */
        public capacitorIsCharged: boolean;

        /** RobotProcessedFeedback batteryLevel. */
        public batteryLevel: number;

        /**
         * Creates a new RobotProcessedFeedback instance using the specified properties.
         * @param [properties] Properties to set
         * @returns RobotProcessedFeedback instance
         */
        public static create(properties?: proto.IRobotProcessedFeedback): proto.RobotProcessedFeedback;

        /**
         * Encodes the specified RobotProcessedFeedback message. Does not implicitly {@link proto.RobotProcessedFeedback.verify|verify} messages.
         * @param message RobotProcessedFeedback message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IRobotProcessedFeedback, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified RobotProcessedFeedback message, length delimited. Does not implicitly {@link proto.RobotProcessedFeedback.verify|verify} messages.
         * @param message RobotProcessedFeedback message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IRobotProcessedFeedback, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a RobotProcessedFeedback message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns RobotProcessedFeedback
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.RobotProcessedFeedback;

        /**
         * Decodes a RobotProcessedFeedback message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns RobotProcessedFeedback
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.RobotProcessedFeedback;

        /**
         * Verifies a RobotProcessedFeedback message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a RobotProcessedFeedback message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns RobotProcessedFeedback
         */
        public static fromObject(object: { [k: string]: any }): proto.RobotProcessedFeedback;

        /**
         * Creates a plain object from a RobotProcessedFeedback message. Also converts values to other types if specified.
         * @param message RobotProcessedFeedback
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.RobotProcessedFeedback, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this RobotProcessedFeedback to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for RobotProcessedFeedback
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a FeedbackOnlyRobot. */
    interface IFeedbackOnlyRobot {

        /** FeedbackOnlyRobot id */
        id?: (number|null);

        /** FeedbackOnlyRobot feedback */
        feedback?: (proto.IRobotProcessedFeedback|null);
    }

    /** Represents a FeedbackOnlyRobot. */
    class FeedbackOnlyRobot implements IFeedbackOnlyRobot {

        /**
         * Constructs a new FeedbackOnlyRobot.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IFeedbackOnlyRobot);

        /** FeedbackOnlyRobot id. */
        public id: number;

        /** FeedbackOnlyRobot feedback. */
        public feedback?: (proto.IRobotProcessedFeedback|null);

        /**
         * Creates a new FeedbackOnlyRobot instance using the specified properties.
         * @param [properties] Properties to set
         * @returns FeedbackOnlyRobot instance
         */
        public static create(properties?: proto.IFeedbackOnlyRobot): proto.FeedbackOnlyRobot;

        /**
         * Encodes the specified FeedbackOnlyRobot message. Does not implicitly {@link proto.FeedbackOnlyRobot.verify|verify} messages.
         * @param message FeedbackOnlyRobot message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IFeedbackOnlyRobot, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified FeedbackOnlyRobot message, length delimited. Does not implicitly {@link proto.FeedbackOnlyRobot.verify|verify} messages.
         * @param message FeedbackOnlyRobot message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IFeedbackOnlyRobot, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a FeedbackOnlyRobot message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns FeedbackOnlyRobot
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.FeedbackOnlyRobot;

        /**
         * Decodes a FeedbackOnlyRobot message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns FeedbackOnlyRobot
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.FeedbackOnlyRobot;

        /**
         * Verifies a FeedbackOnlyRobot message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a FeedbackOnlyRobot message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns FeedbackOnlyRobot
         */
        public static fromObject(object: { [k: string]: any }): proto.FeedbackOnlyRobot;

        /**
         * Creates a plain object from a FeedbackOnlyRobot message. Also converts values to other types if specified.
         * @param message FeedbackOnlyRobot
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.FeedbackOnlyRobot, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this FeedbackOnlyRobot to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for FeedbackOnlyRobot
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a RobotParameters. */
    interface IRobotParameters {

        /** RobotParameters radius */
        radius?: (number|null);

        /** RobotParameters height */
        height?: (number|null);

        /** RobotParameters frontWidth */
        frontWidth?: (number|null);

        /** RobotParameters dribblerWidth */
        dribblerWidth?: (number|null);

        /** RobotParameters yawOffset */
        yawOffset?: (number|null);
    }

    /** Represents a RobotParameters. */
    class RobotParameters implements IRobotParameters {

        /**
         * Constructs a new RobotParameters.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IRobotParameters);

        /** RobotParameters radius. */
        public radius: number;

        /** RobotParameters height. */
        public height: number;

        /** RobotParameters frontWidth. */
        public frontWidth: number;

        /** RobotParameters dribblerWidth. */
        public dribblerWidth: number;

        /** RobotParameters yawOffset. */
        public yawOffset: number;

        /**
         * Creates a new RobotParameters instance using the specified properties.
         * @param [properties] Properties to set
         * @returns RobotParameters instance
         */
        public static create(properties?: proto.IRobotParameters): proto.RobotParameters;

        /**
         * Encodes the specified RobotParameters message. Does not implicitly {@link proto.RobotParameters.verify|verify} messages.
         * @param message RobotParameters message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IRobotParameters, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified RobotParameters message, length delimited. Does not implicitly {@link proto.RobotParameters.verify|verify} messages.
         * @param message RobotParameters message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IRobotParameters, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a RobotParameters message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns RobotParameters
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.RobotParameters;

        /**
         * Decodes a RobotParameters message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns RobotParameters
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.RobotParameters;

        /**
         * Verifies a RobotParameters message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a RobotParameters message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns RobotParameters
         */
        public static fromObject(object: { [k: string]: any }): proto.RobotParameters;

        /**
         * Creates a plain object from a RobotParameters message. Also converts values to other types if specified.
         * @param message RobotParameters
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.RobotParameters, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this RobotParameters to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for RobotParameters
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a TeamParameters. */
    interface ITeamParameters {

        /** TeamParameters didChange */
        didChange?: (boolean|null);

        /** TeamParameters parameters */
        parameters?: (proto.IRobotParameters|null);
    }

    /** Represents a TeamParameters. */
    class TeamParameters implements ITeamParameters {

        /**
         * Constructs a new TeamParameters.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.ITeamParameters);

        /** TeamParameters didChange. */
        public didChange: boolean;

        /** TeamParameters parameters. */
        public parameters?: (proto.IRobotParameters|null);

        /**
         * Creates a new TeamParameters instance using the specified properties.
         * @param [properties] Properties to set
         * @returns TeamParameters instance
         */
        public static create(properties?: proto.ITeamParameters): proto.TeamParameters;

        /**
         * Encodes the specified TeamParameters message. Does not implicitly {@link proto.TeamParameters.verify|verify} messages.
         * @param message TeamParameters message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.ITeamParameters, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified TeamParameters message, length delimited. Does not implicitly {@link proto.TeamParameters.verify|verify} messages.
         * @param message TeamParameters message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.ITeamParameters, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a TeamParameters message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns TeamParameters
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.TeamParameters;

        /**
         * Decodes a TeamParameters message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns TeamParameters
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.TeamParameters;

        /**
         * Verifies a TeamParameters message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a TeamParameters message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns TeamParameters
         */
        public static fromObject(object: { [k: string]: any }): proto.TeamParameters;

        /**
         * Creates a plain object from a TeamParameters message. Also converts values to other types if specified.
         * @param message TeamParameters
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.TeamParameters, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this TeamParameters to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for TeamParameters
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a SSL_WrapperPacket. */
    interface ISSL_WrapperPacket {

        /** SSL_WrapperPacket detection */
        detection?: (proto.ISSL_DetectionFrame|null);

        /** SSL_WrapperPacket geometry */
        geometry?: (proto.ISSL_GeometryData|null);
    }

    /** Represents a SSL_WrapperPacket. */
    class SSL_WrapperPacket implements ISSL_WrapperPacket {

        /**
         * Constructs a new SSL_WrapperPacket.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.ISSL_WrapperPacket);

        /** SSL_WrapperPacket detection. */
        public detection?: (proto.ISSL_DetectionFrame|null);

        /** SSL_WrapperPacket geometry. */
        public geometry?: (proto.ISSL_GeometryData|null);

        /**
         * Creates a new SSL_WrapperPacket instance using the specified properties.
         * @param [properties] Properties to set
         * @returns SSL_WrapperPacket instance
         */
        public static create(properties?: proto.ISSL_WrapperPacket): proto.SSL_WrapperPacket;

        /**
         * Encodes the specified SSL_WrapperPacket message. Does not implicitly {@link proto.SSL_WrapperPacket.verify|verify} messages.
         * @param message SSL_WrapperPacket message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.ISSL_WrapperPacket, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified SSL_WrapperPacket message, length delimited. Does not implicitly {@link proto.SSL_WrapperPacket.verify|verify} messages.
         * @param message SSL_WrapperPacket message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.ISSL_WrapperPacket, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a SSL_WrapperPacket message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns SSL_WrapperPacket
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.SSL_WrapperPacket;

        /**
         * Decodes a SSL_WrapperPacket message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns SSL_WrapperPacket
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.SSL_WrapperPacket;

        /**
         * Verifies a SSL_WrapperPacket message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a SSL_WrapperPacket message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns SSL_WrapperPacket
         */
        public static fromObject(object: { [k: string]: any }): proto.SSL_WrapperPacket;

        /**
         * Creates a plain object from a SSL_WrapperPacket message. Also converts values to other types if specified.
         * @param message SSL_WrapperPacket
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.SSL_WrapperPacket, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this SSL_WrapperPacket to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for SSL_WrapperPacket
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a SSL_DetectionBall. */
    interface ISSL_DetectionBall {

        /** SSL_DetectionBall confidence */
        confidence: number;

        /** SSL_DetectionBall area */
        area?: (number|null);

        /** SSL_DetectionBall x */
        x: number;

        /** SSL_DetectionBall y */
        y: number;

        /** SSL_DetectionBall z */
        z?: (number|null);

        /** SSL_DetectionBall pixelX */
        pixelX: number;

        /** SSL_DetectionBall pixelY */
        pixelY: number;
    }

    /** Represents a SSL_DetectionBall. */
    class SSL_DetectionBall implements ISSL_DetectionBall {

        /**
         * Constructs a new SSL_DetectionBall.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.ISSL_DetectionBall);

        /** SSL_DetectionBall confidence. */
        public confidence: number;

        /** SSL_DetectionBall area. */
        public area: number;

        /** SSL_DetectionBall x. */
        public x: number;

        /** SSL_DetectionBall y. */
        public y: number;

        /** SSL_DetectionBall z. */
        public z: number;

        /** SSL_DetectionBall pixelX. */
        public pixelX: number;

        /** SSL_DetectionBall pixelY. */
        public pixelY: number;

        /**
         * Creates a new SSL_DetectionBall instance using the specified properties.
         * @param [properties] Properties to set
         * @returns SSL_DetectionBall instance
         */
        public static create(properties?: proto.ISSL_DetectionBall): proto.SSL_DetectionBall;

        /**
         * Encodes the specified SSL_DetectionBall message. Does not implicitly {@link proto.SSL_DetectionBall.verify|verify} messages.
         * @param message SSL_DetectionBall message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.ISSL_DetectionBall, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified SSL_DetectionBall message, length delimited. Does not implicitly {@link proto.SSL_DetectionBall.verify|verify} messages.
         * @param message SSL_DetectionBall message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.ISSL_DetectionBall, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a SSL_DetectionBall message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns SSL_DetectionBall
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.SSL_DetectionBall;

        /**
         * Decodes a SSL_DetectionBall message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns SSL_DetectionBall
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.SSL_DetectionBall;

        /**
         * Verifies a SSL_DetectionBall message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a SSL_DetectionBall message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns SSL_DetectionBall
         */
        public static fromObject(object: { [k: string]: any }): proto.SSL_DetectionBall;

        /**
         * Creates a plain object from a SSL_DetectionBall message. Also converts values to other types if specified.
         * @param message SSL_DetectionBall
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.SSL_DetectionBall, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this SSL_DetectionBall to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for SSL_DetectionBall
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a SSL_DetectionRobot. */
    interface ISSL_DetectionRobot {

        /** SSL_DetectionRobot confidence */
        confidence: number;

        /** SSL_DetectionRobot robotId */
        robotId?: (number|null);

        /** SSL_DetectionRobot x */
        x: number;

        /** SSL_DetectionRobot y */
        y: number;

        /** SSL_DetectionRobot orientation */
        orientation?: (number|null);

        /** SSL_DetectionRobot pixelX */
        pixelX: number;

        /** SSL_DetectionRobot pixelY */
        pixelY: number;

        /** SSL_DetectionRobot height */
        height?: (number|null);
    }

    /** Represents a SSL_DetectionRobot. */
    class SSL_DetectionRobot implements ISSL_DetectionRobot {

        /**
         * Constructs a new SSL_DetectionRobot.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.ISSL_DetectionRobot);

        /** SSL_DetectionRobot confidence. */
        public confidence: number;

        /** SSL_DetectionRobot robotId. */
        public robotId: number;

        /** SSL_DetectionRobot x. */
        public x: number;

        /** SSL_DetectionRobot y. */
        public y: number;

        /** SSL_DetectionRobot orientation. */
        public orientation: number;

        /** SSL_DetectionRobot pixelX. */
        public pixelX: number;

        /** SSL_DetectionRobot pixelY. */
        public pixelY: number;

        /** SSL_DetectionRobot height. */
        public height: number;

        /**
         * Creates a new SSL_DetectionRobot instance using the specified properties.
         * @param [properties] Properties to set
         * @returns SSL_DetectionRobot instance
         */
        public static create(properties?: proto.ISSL_DetectionRobot): proto.SSL_DetectionRobot;

        /**
         * Encodes the specified SSL_DetectionRobot message. Does not implicitly {@link proto.SSL_DetectionRobot.verify|verify} messages.
         * @param message SSL_DetectionRobot message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.ISSL_DetectionRobot, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified SSL_DetectionRobot message, length delimited. Does not implicitly {@link proto.SSL_DetectionRobot.verify|verify} messages.
         * @param message SSL_DetectionRobot message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.ISSL_DetectionRobot, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a SSL_DetectionRobot message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns SSL_DetectionRobot
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.SSL_DetectionRobot;

        /**
         * Decodes a SSL_DetectionRobot message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns SSL_DetectionRobot
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.SSL_DetectionRobot;

        /**
         * Verifies a SSL_DetectionRobot message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a SSL_DetectionRobot message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns SSL_DetectionRobot
         */
        public static fromObject(object: { [k: string]: any }): proto.SSL_DetectionRobot;

        /**
         * Creates a plain object from a SSL_DetectionRobot message. Also converts values to other types if specified.
         * @param message SSL_DetectionRobot
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.SSL_DetectionRobot, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this SSL_DetectionRobot to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for SSL_DetectionRobot
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a SSL_DetectionFrame. */
    interface ISSL_DetectionFrame {

        /** SSL_DetectionFrame frameNumber */
        frameNumber: number;

        /** SSL_DetectionFrame tCapture */
        tCapture: number;

        /** SSL_DetectionFrame tSent */
        tSent: number;

        /** SSL_DetectionFrame cameraId */
        cameraId: number;

        /** SSL_DetectionFrame balls */
        balls?: (proto.ISSL_DetectionBall[]|null);

        /** SSL_DetectionFrame robotsYellow */
        robotsYellow?: (proto.ISSL_DetectionRobot[]|null);

        /** SSL_DetectionFrame robotsBlue */
        robotsBlue?: (proto.ISSL_DetectionRobot[]|null);
    }

    /** Represents a SSL_DetectionFrame. */
    class SSL_DetectionFrame implements ISSL_DetectionFrame {

        /**
         * Constructs a new SSL_DetectionFrame.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.ISSL_DetectionFrame);

        /** SSL_DetectionFrame frameNumber. */
        public frameNumber: number;

        /** SSL_DetectionFrame tCapture. */
        public tCapture: number;

        /** SSL_DetectionFrame tSent. */
        public tSent: number;

        /** SSL_DetectionFrame cameraId. */
        public cameraId: number;

        /** SSL_DetectionFrame balls. */
        public balls: proto.ISSL_DetectionBall[];

        /** SSL_DetectionFrame robotsYellow. */
        public robotsYellow: proto.ISSL_DetectionRobot[];

        /** SSL_DetectionFrame robotsBlue. */
        public robotsBlue: proto.ISSL_DetectionRobot[];

        /**
         * Creates a new SSL_DetectionFrame instance using the specified properties.
         * @param [properties] Properties to set
         * @returns SSL_DetectionFrame instance
         */
        public static create(properties?: proto.ISSL_DetectionFrame): proto.SSL_DetectionFrame;

        /**
         * Encodes the specified SSL_DetectionFrame message. Does not implicitly {@link proto.SSL_DetectionFrame.verify|verify} messages.
         * @param message SSL_DetectionFrame message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.ISSL_DetectionFrame, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified SSL_DetectionFrame message, length delimited. Does not implicitly {@link proto.SSL_DetectionFrame.verify|verify} messages.
         * @param message SSL_DetectionFrame message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.ISSL_DetectionFrame, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a SSL_DetectionFrame message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns SSL_DetectionFrame
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.SSL_DetectionFrame;

        /**
         * Decodes a SSL_DetectionFrame message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns SSL_DetectionFrame
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.SSL_DetectionFrame;

        /**
         * Verifies a SSL_DetectionFrame message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a SSL_DetectionFrame message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns SSL_DetectionFrame
         */
        public static fromObject(object: { [k: string]: any }): proto.SSL_DetectionFrame;

        /**
         * Creates a plain object from a SSL_DetectionFrame message. Also converts values to other types if specified.
         * @param message SSL_DetectionFrame
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.SSL_DetectionFrame, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this SSL_DetectionFrame to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for SSL_DetectionFrame
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a SSL_FieldLineSegment. */
    interface ISSL_FieldLineSegment {

        /** SSL_FieldLineSegment name */
        name: string;

        /** SSL_FieldLineSegment p1 */
        p1: proto.IVector2f;

        /** SSL_FieldLineSegment p2 */
        p2: proto.IVector2f;

        /** SSL_FieldLineSegment thickness */
        thickness: number;

        /** SSL_FieldLineSegment type */
        type?: (proto.SSL_FieldShapeType|null);
    }

    /** Represents a SSL_FieldLineSegment. */
    class SSL_FieldLineSegment implements ISSL_FieldLineSegment {

        /**
         * Constructs a new SSL_FieldLineSegment.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.ISSL_FieldLineSegment);

        /** SSL_FieldLineSegment name. */
        public name: string;

        /** SSL_FieldLineSegment p1. */
        public p1: proto.IVector2f;

        /** SSL_FieldLineSegment p2. */
        public p2: proto.IVector2f;

        /** SSL_FieldLineSegment thickness. */
        public thickness: number;

        /** SSL_FieldLineSegment type. */
        public type: proto.SSL_FieldShapeType;

        /**
         * Creates a new SSL_FieldLineSegment instance using the specified properties.
         * @param [properties] Properties to set
         * @returns SSL_FieldLineSegment instance
         */
        public static create(properties?: proto.ISSL_FieldLineSegment): proto.SSL_FieldLineSegment;

        /**
         * Encodes the specified SSL_FieldLineSegment message. Does not implicitly {@link proto.SSL_FieldLineSegment.verify|verify} messages.
         * @param message SSL_FieldLineSegment message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.ISSL_FieldLineSegment, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified SSL_FieldLineSegment message, length delimited. Does not implicitly {@link proto.SSL_FieldLineSegment.verify|verify} messages.
         * @param message SSL_FieldLineSegment message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.ISSL_FieldLineSegment, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a SSL_FieldLineSegment message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns SSL_FieldLineSegment
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.SSL_FieldLineSegment;

        /**
         * Decodes a SSL_FieldLineSegment message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns SSL_FieldLineSegment
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.SSL_FieldLineSegment;

        /**
         * Verifies a SSL_FieldLineSegment message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a SSL_FieldLineSegment message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns SSL_FieldLineSegment
         */
        public static fromObject(object: { [k: string]: any }): proto.SSL_FieldLineSegment;

        /**
         * Creates a plain object from a SSL_FieldLineSegment message. Also converts values to other types if specified.
         * @param message SSL_FieldLineSegment
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.SSL_FieldLineSegment, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this SSL_FieldLineSegment to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for SSL_FieldLineSegment
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a SSL_FieldCircularArc. */
    interface ISSL_FieldCircularArc {

        /** SSL_FieldCircularArc name */
        name: string;

        /** SSL_FieldCircularArc center */
        center: proto.IVector2f;

        /** SSL_FieldCircularArc radius */
        radius: number;

        /** SSL_FieldCircularArc a1 */
        a1: number;

        /** SSL_FieldCircularArc a2 */
        a2: number;

        /** SSL_FieldCircularArc thickness */
        thickness: number;

        /** SSL_FieldCircularArc type */
        type?: (proto.SSL_FieldShapeType|null);
    }

    /** Represents a SSL_FieldCircularArc. */
    class SSL_FieldCircularArc implements ISSL_FieldCircularArc {

        /**
         * Constructs a new SSL_FieldCircularArc.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.ISSL_FieldCircularArc);

        /** SSL_FieldCircularArc name. */
        public name: string;

        /** SSL_FieldCircularArc center. */
        public center: proto.IVector2f;

        /** SSL_FieldCircularArc radius. */
        public radius: number;

        /** SSL_FieldCircularArc a1. */
        public a1: number;

        /** SSL_FieldCircularArc a2. */
        public a2: number;

        /** SSL_FieldCircularArc thickness. */
        public thickness: number;

        /** SSL_FieldCircularArc type. */
        public type: proto.SSL_FieldShapeType;

        /**
         * Creates a new SSL_FieldCircularArc instance using the specified properties.
         * @param [properties] Properties to set
         * @returns SSL_FieldCircularArc instance
         */
        public static create(properties?: proto.ISSL_FieldCircularArc): proto.SSL_FieldCircularArc;

        /**
         * Encodes the specified SSL_FieldCircularArc message. Does not implicitly {@link proto.SSL_FieldCircularArc.verify|verify} messages.
         * @param message SSL_FieldCircularArc message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.ISSL_FieldCircularArc, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified SSL_FieldCircularArc message, length delimited. Does not implicitly {@link proto.SSL_FieldCircularArc.verify|verify} messages.
         * @param message SSL_FieldCircularArc message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.ISSL_FieldCircularArc, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a SSL_FieldCircularArc message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns SSL_FieldCircularArc
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.SSL_FieldCircularArc;

        /**
         * Decodes a SSL_FieldCircularArc message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns SSL_FieldCircularArc
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.SSL_FieldCircularArc;

        /**
         * Verifies a SSL_FieldCircularArc message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a SSL_FieldCircularArc message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns SSL_FieldCircularArc
         */
        public static fromObject(object: { [k: string]: any }): proto.SSL_FieldCircularArc;

        /**
         * Creates a plain object from a SSL_FieldCircularArc message. Also converts values to other types if specified.
         * @param message SSL_FieldCircularArc
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.SSL_FieldCircularArc, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this SSL_FieldCircularArc to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for SSL_FieldCircularArc
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a SSL_GeometryFieldSize. */
    interface ISSL_GeometryFieldSize {

        /** SSL_GeometryFieldSize fieldLength */
        fieldLength: number;

        /** SSL_GeometryFieldSize fieldWidth */
        fieldWidth: number;

        /** SSL_GeometryFieldSize goalWidth */
        goalWidth: number;

        /** SSL_GeometryFieldSize goalDepth */
        goalDepth: number;

        /** SSL_GeometryFieldSize boundaryWidth */
        boundaryWidth: number;

        /** SSL_GeometryFieldSize fieldLines */
        fieldLines?: (proto.ISSL_FieldLineSegment[]|null);

        /** SSL_GeometryFieldSize fieldArcs */
        fieldArcs?: (proto.ISSL_FieldCircularArc[]|null);

        /** SSL_GeometryFieldSize penaltyAreaDepth */
        penaltyAreaDepth?: (number|null);

        /** SSL_GeometryFieldSize penaltyAreaWidth */
        penaltyAreaWidth?: (number|null);

        /** SSL_GeometryFieldSize centerCircleRadius */
        centerCircleRadius?: (number|null);

        /** SSL_GeometryFieldSize lineThickness */
        lineThickness?: (number|null);

        /** SSL_GeometryFieldSize goalCenterToPenaltyMark */
        goalCenterToPenaltyMark?: (number|null);

        /** SSL_GeometryFieldSize goalHeight */
        goalHeight?: (number|null);

        /** SSL_GeometryFieldSize ballRadius */
        ballRadius?: (number|null);

        /** SSL_GeometryFieldSize maxRobotRadius */
        maxRobotRadius?: (number|null);
    }

    /** Represents a SSL_GeometryFieldSize. */
    class SSL_GeometryFieldSize implements ISSL_GeometryFieldSize {

        /**
         * Constructs a new SSL_GeometryFieldSize.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.ISSL_GeometryFieldSize);

        /** SSL_GeometryFieldSize fieldLength. */
        public fieldLength: number;

        /** SSL_GeometryFieldSize fieldWidth. */
        public fieldWidth: number;

        /** SSL_GeometryFieldSize goalWidth. */
        public goalWidth: number;

        /** SSL_GeometryFieldSize goalDepth. */
        public goalDepth: number;

        /** SSL_GeometryFieldSize boundaryWidth. */
        public boundaryWidth: number;

        /** SSL_GeometryFieldSize fieldLines. */
        public fieldLines: proto.ISSL_FieldLineSegment[];

        /** SSL_GeometryFieldSize fieldArcs. */
        public fieldArcs: proto.ISSL_FieldCircularArc[];

        /** SSL_GeometryFieldSize penaltyAreaDepth. */
        public penaltyAreaDepth: number;

        /** SSL_GeometryFieldSize penaltyAreaWidth. */
        public penaltyAreaWidth: number;

        /** SSL_GeometryFieldSize centerCircleRadius. */
        public centerCircleRadius: number;

        /** SSL_GeometryFieldSize lineThickness. */
        public lineThickness: number;

        /** SSL_GeometryFieldSize goalCenterToPenaltyMark. */
        public goalCenterToPenaltyMark: number;

        /** SSL_GeometryFieldSize goalHeight. */
        public goalHeight: number;

        /** SSL_GeometryFieldSize ballRadius. */
        public ballRadius: number;

        /** SSL_GeometryFieldSize maxRobotRadius. */
        public maxRobotRadius: number;

        /**
         * Creates a new SSL_GeometryFieldSize instance using the specified properties.
         * @param [properties] Properties to set
         * @returns SSL_GeometryFieldSize instance
         */
        public static create(properties?: proto.ISSL_GeometryFieldSize): proto.SSL_GeometryFieldSize;

        /**
         * Encodes the specified SSL_GeometryFieldSize message. Does not implicitly {@link proto.SSL_GeometryFieldSize.verify|verify} messages.
         * @param message SSL_GeometryFieldSize message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.ISSL_GeometryFieldSize, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified SSL_GeometryFieldSize message, length delimited. Does not implicitly {@link proto.SSL_GeometryFieldSize.verify|verify} messages.
         * @param message SSL_GeometryFieldSize message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.ISSL_GeometryFieldSize, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a SSL_GeometryFieldSize message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns SSL_GeometryFieldSize
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.SSL_GeometryFieldSize;

        /**
         * Decodes a SSL_GeometryFieldSize message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns SSL_GeometryFieldSize
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.SSL_GeometryFieldSize;

        /**
         * Verifies a SSL_GeometryFieldSize message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a SSL_GeometryFieldSize message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns SSL_GeometryFieldSize
         */
        public static fromObject(object: { [k: string]: any }): proto.SSL_GeometryFieldSize;

        /**
         * Creates a plain object from a SSL_GeometryFieldSize message. Also converts values to other types if specified.
         * @param message SSL_GeometryFieldSize
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.SSL_GeometryFieldSize, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this SSL_GeometryFieldSize to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for SSL_GeometryFieldSize
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a SSL_GeometryCameraCalibration. */
    interface ISSL_GeometryCameraCalibration {

        /** SSL_GeometryCameraCalibration cameraId */
        cameraId: number;

        /** SSL_GeometryCameraCalibration focalLength */
        focalLength: number;

        /** SSL_GeometryCameraCalibration principalPointX */
        principalPointX: number;

        /** SSL_GeometryCameraCalibration principalPointY */
        principalPointY: number;

        /** SSL_GeometryCameraCalibration distortion */
        distortion: number;

        /** SSL_GeometryCameraCalibration q0 */
        q0: number;

        /** SSL_GeometryCameraCalibration q1 */
        q1: number;

        /** SSL_GeometryCameraCalibration q2 */
        q2: number;

        /** SSL_GeometryCameraCalibration q3 */
        q3: number;

        /** SSL_GeometryCameraCalibration tx */
        tx: number;

        /** SSL_GeometryCameraCalibration ty */
        ty: number;

        /** SSL_GeometryCameraCalibration tz */
        tz: number;

        /** SSL_GeometryCameraCalibration derivedCameraWorldTx */
        derivedCameraWorldTx?: (number|null);

        /** SSL_GeometryCameraCalibration derivedCameraWorldTy */
        derivedCameraWorldTy?: (number|null);

        /** SSL_GeometryCameraCalibration derivedCameraWorldTz */
        derivedCameraWorldTz?: (number|null);

        /** SSL_GeometryCameraCalibration pixelImageWidth */
        pixelImageWidth?: (number|null);

        /** SSL_GeometryCameraCalibration pixelImageHeight */
        pixelImageHeight?: (number|null);
    }

    /** Represents a SSL_GeometryCameraCalibration. */
    class SSL_GeometryCameraCalibration implements ISSL_GeometryCameraCalibration {

        /**
         * Constructs a new SSL_GeometryCameraCalibration.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.ISSL_GeometryCameraCalibration);

        /** SSL_GeometryCameraCalibration cameraId. */
        public cameraId: number;

        /** SSL_GeometryCameraCalibration focalLength. */
        public focalLength: number;

        /** SSL_GeometryCameraCalibration principalPointX. */
        public principalPointX: number;

        /** SSL_GeometryCameraCalibration principalPointY. */
        public principalPointY: number;

        /** SSL_GeometryCameraCalibration distortion. */
        public distortion: number;

        /** SSL_GeometryCameraCalibration q0. */
        public q0: number;

        /** SSL_GeometryCameraCalibration q1. */
        public q1: number;

        /** SSL_GeometryCameraCalibration q2. */
        public q2: number;

        /** SSL_GeometryCameraCalibration q3. */
        public q3: number;

        /** SSL_GeometryCameraCalibration tx. */
        public tx: number;

        /** SSL_GeometryCameraCalibration ty. */
        public ty: number;

        /** SSL_GeometryCameraCalibration tz. */
        public tz: number;

        /** SSL_GeometryCameraCalibration derivedCameraWorldTx. */
        public derivedCameraWorldTx: number;

        /** SSL_GeometryCameraCalibration derivedCameraWorldTy. */
        public derivedCameraWorldTy: number;

        /** SSL_GeometryCameraCalibration derivedCameraWorldTz. */
        public derivedCameraWorldTz: number;

        /** SSL_GeometryCameraCalibration pixelImageWidth. */
        public pixelImageWidth: number;

        /** SSL_GeometryCameraCalibration pixelImageHeight. */
        public pixelImageHeight: number;

        /**
         * Creates a new SSL_GeometryCameraCalibration instance using the specified properties.
         * @param [properties] Properties to set
         * @returns SSL_GeometryCameraCalibration instance
         */
        public static create(properties?: proto.ISSL_GeometryCameraCalibration): proto.SSL_GeometryCameraCalibration;

        /**
         * Encodes the specified SSL_GeometryCameraCalibration message. Does not implicitly {@link proto.SSL_GeometryCameraCalibration.verify|verify} messages.
         * @param message SSL_GeometryCameraCalibration message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.ISSL_GeometryCameraCalibration, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified SSL_GeometryCameraCalibration message, length delimited. Does not implicitly {@link proto.SSL_GeometryCameraCalibration.verify|verify} messages.
         * @param message SSL_GeometryCameraCalibration message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.ISSL_GeometryCameraCalibration, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a SSL_GeometryCameraCalibration message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns SSL_GeometryCameraCalibration
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.SSL_GeometryCameraCalibration;

        /**
         * Decodes a SSL_GeometryCameraCalibration message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns SSL_GeometryCameraCalibration
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.SSL_GeometryCameraCalibration;

        /**
         * Verifies a SSL_GeometryCameraCalibration message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a SSL_GeometryCameraCalibration message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns SSL_GeometryCameraCalibration
         */
        public static fromObject(object: { [k: string]: any }): proto.SSL_GeometryCameraCalibration;

        /**
         * Creates a plain object from a SSL_GeometryCameraCalibration message. Also converts values to other types if specified.
         * @param message SSL_GeometryCameraCalibration
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.SSL_GeometryCameraCalibration, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this SSL_GeometryCameraCalibration to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for SSL_GeometryCameraCalibration
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a SSL_BallModelStraightTwoPhase. */
    interface ISSL_BallModelStraightTwoPhase {

        /** SSL_BallModelStraightTwoPhase accSlide */
        accSlide: number;

        /** SSL_BallModelStraightTwoPhase accRoll */
        accRoll: number;

        /** SSL_BallModelStraightTwoPhase kSwitch */
        kSwitch: number;
    }

    /** Represents a SSL_BallModelStraightTwoPhase. */
    class SSL_BallModelStraightTwoPhase implements ISSL_BallModelStraightTwoPhase {

        /**
         * Constructs a new SSL_BallModelStraightTwoPhase.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.ISSL_BallModelStraightTwoPhase);

        /** SSL_BallModelStraightTwoPhase accSlide. */
        public accSlide: number;

        /** SSL_BallModelStraightTwoPhase accRoll. */
        public accRoll: number;

        /** SSL_BallModelStraightTwoPhase kSwitch. */
        public kSwitch: number;

        /**
         * Creates a new SSL_BallModelStraightTwoPhase instance using the specified properties.
         * @param [properties] Properties to set
         * @returns SSL_BallModelStraightTwoPhase instance
         */
        public static create(properties?: proto.ISSL_BallModelStraightTwoPhase): proto.SSL_BallModelStraightTwoPhase;

        /**
         * Encodes the specified SSL_BallModelStraightTwoPhase message. Does not implicitly {@link proto.SSL_BallModelStraightTwoPhase.verify|verify} messages.
         * @param message SSL_BallModelStraightTwoPhase message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.ISSL_BallModelStraightTwoPhase, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified SSL_BallModelStraightTwoPhase message, length delimited. Does not implicitly {@link proto.SSL_BallModelStraightTwoPhase.verify|verify} messages.
         * @param message SSL_BallModelStraightTwoPhase message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.ISSL_BallModelStraightTwoPhase, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a SSL_BallModelStraightTwoPhase message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns SSL_BallModelStraightTwoPhase
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.SSL_BallModelStraightTwoPhase;

        /**
         * Decodes a SSL_BallModelStraightTwoPhase message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns SSL_BallModelStraightTwoPhase
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.SSL_BallModelStraightTwoPhase;

        /**
         * Verifies a SSL_BallModelStraightTwoPhase message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a SSL_BallModelStraightTwoPhase message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns SSL_BallModelStraightTwoPhase
         */
        public static fromObject(object: { [k: string]: any }): proto.SSL_BallModelStraightTwoPhase;

        /**
         * Creates a plain object from a SSL_BallModelStraightTwoPhase message. Also converts values to other types if specified.
         * @param message SSL_BallModelStraightTwoPhase
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.SSL_BallModelStraightTwoPhase, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this SSL_BallModelStraightTwoPhase to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for SSL_BallModelStraightTwoPhase
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a SSL_BallModelChipFixedLoss. */
    interface ISSL_BallModelChipFixedLoss {

        /** SSL_BallModelChipFixedLoss dampingXyFirstHop */
        dampingXyFirstHop: number;

        /** SSL_BallModelChipFixedLoss dampingXyOtherHops */
        dampingXyOtherHops: number;

        /** SSL_BallModelChipFixedLoss dampingZ */
        dampingZ: number;
    }

    /** Represents a SSL_BallModelChipFixedLoss. */
    class SSL_BallModelChipFixedLoss implements ISSL_BallModelChipFixedLoss {

        /**
         * Constructs a new SSL_BallModelChipFixedLoss.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.ISSL_BallModelChipFixedLoss);

        /** SSL_BallModelChipFixedLoss dampingXyFirstHop. */
        public dampingXyFirstHop: number;

        /** SSL_BallModelChipFixedLoss dampingXyOtherHops. */
        public dampingXyOtherHops: number;

        /** SSL_BallModelChipFixedLoss dampingZ. */
        public dampingZ: number;

        /**
         * Creates a new SSL_BallModelChipFixedLoss instance using the specified properties.
         * @param [properties] Properties to set
         * @returns SSL_BallModelChipFixedLoss instance
         */
        public static create(properties?: proto.ISSL_BallModelChipFixedLoss): proto.SSL_BallModelChipFixedLoss;

        /**
         * Encodes the specified SSL_BallModelChipFixedLoss message. Does not implicitly {@link proto.SSL_BallModelChipFixedLoss.verify|verify} messages.
         * @param message SSL_BallModelChipFixedLoss message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.ISSL_BallModelChipFixedLoss, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified SSL_BallModelChipFixedLoss message, length delimited. Does not implicitly {@link proto.SSL_BallModelChipFixedLoss.verify|verify} messages.
         * @param message SSL_BallModelChipFixedLoss message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.ISSL_BallModelChipFixedLoss, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a SSL_BallModelChipFixedLoss message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns SSL_BallModelChipFixedLoss
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.SSL_BallModelChipFixedLoss;

        /**
         * Decodes a SSL_BallModelChipFixedLoss message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns SSL_BallModelChipFixedLoss
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.SSL_BallModelChipFixedLoss;

        /**
         * Verifies a SSL_BallModelChipFixedLoss message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a SSL_BallModelChipFixedLoss message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns SSL_BallModelChipFixedLoss
         */
        public static fromObject(object: { [k: string]: any }): proto.SSL_BallModelChipFixedLoss;

        /**
         * Creates a plain object from a SSL_BallModelChipFixedLoss message. Also converts values to other types if specified.
         * @param message SSL_BallModelChipFixedLoss
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.SSL_BallModelChipFixedLoss, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this SSL_BallModelChipFixedLoss to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for SSL_BallModelChipFixedLoss
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a SSL_GeometryModels. */
    interface ISSL_GeometryModels {

        /** SSL_GeometryModels straightTwoPhase */
        straightTwoPhase?: (proto.ISSL_BallModelStraightTwoPhase|null);

        /** SSL_GeometryModels chipFixedLoss */
        chipFixedLoss?: (proto.ISSL_BallModelChipFixedLoss|null);
    }

    /** Represents a SSL_GeometryModels. */
    class SSL_GeometryModels implements ISSL_GeometryModels {

        /**
         * Constructs a new SSL_GeometryModels.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.ISSL_GeometryModels);

        /** SSL_GeometryModels straightTwoPhase. */
        public straightTwoPhase?: (proto.ISSL_BallModelStraightTwoPhase|null);

        /** SSL_GeometryModels chipFixedLoss. */
        public chipFixedLoss?: (proto.ISSL_BallModelChipFixedLoss|null);

        /**
         * Creates a new SSL_GeometryModels instance using the specified properties.
         * @param [properties] Properties to set
         * @returns SSL_GeometryModels instance
         */
        public static create(properties?: proto.ISSL_GeometryModels): proto.SSL_GeometryModels;

        /**
         * Encodes the specified SSL_GeometryModels message. Does not implicitly {@link proto.SSL_GeometryModels.verify|verify} messages.
         * @param message SSL_GeometryModels message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.ISSL_GeometryModels, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified SSL_GeometryModels message, length delimited. Does not implicitly {@link proto.SSL_GeometryModels.verify|verify} messages.
         * @param message SSL_GeometryModels message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.ISSL_GeometryModels, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a SSL_GeometryModels message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns SSL_GeometryModels
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.SSL_GeometryModels;

        /**
         * Decodes a SSL_GeometryModels message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns SSL_GeometryModels
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.SSL_GeometryModels;

        /**
         * Verifies a SSL_GeometryModels message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a SSL_GeometryModels message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns SSL_GeometryModels
         */
        public static fromObject(object: { [k: string]: any }): proto.SSL_GeometryModels;

        /**
         * Creates a plain object from a SSL_GeometryModels message. Also converts values to other types if specified.
         * @param message SSL_GeometryModels
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.SSL_GeometryModels, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this SSL_GeometryModels to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for SSL_GeometryModels
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a SSL_GeometryData. */
    interface ISSL_GeometryData {

        /** SSL_GeometryData field */
        field: proto.ISSL_GeometryFieldSize;

        /** SSL_GeometryData calib */
        calib?: (proto.ISSL_GeometryCameraCalibration[]|null);

        /** SSL_GeometryData models */
        models?: (proto.ISSL_GeometryModels|null);
    }

    /** Represents a SSL_GeometryData. */
    class SSL_GeometryData implements ISSL_GeometryData {

        /**
         * Constructs a new SSL_GeometryData.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.ISSL_GeometryData);

        /** SSL_GeometryData field. */
        public field: proto.ISSL_GeometryFieldSize;

        /** SSL_GeometryData calib. */
        public calib: proto.ISSL_GeometryCameraCalibration[];

        /** SSL_GeometryData models. */
        public models?: (proto.ISSL_GeometryModels|null);

        /**
         * Creates a new SSL_GeometryData instance using the specified properties.
         * @param [properties] Properties to set
         * @returns SSL_GeometryData instance
         */
        public static create(properties?: proto.ISSL_GeometryData): proto.SSL_GeometryData;

        /**
         * Encodes the specified SSL_GeometryData message. Does not implicitly {@link proto.SSL_GeometryData.verify|verify} messages.
         * @param message SSL_GeometryData message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.ISSL_GeometryData, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified SSL_GeometryData message, length delimited. Does not implicitly {@link proto.SSL_GeometryData.verify|verify} messages.
         * @param message SSL_GeometryData message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.ISSL_GeometryData, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a SSL_GeometryData message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns SSL_GeometryData
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.SSL_GeometryData;

        /**
         * Decodes a SSL_GeometryData message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns SSL_GeometryData
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.SSL_GeometryData;

        /**
         * Verifies a SSL_GeometryData message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a SSL_GeometryData message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns SSL_GeometryData
         */
        public static fromObject(object: { [k: string]: any }): proto.SSL_GeometryData;

        /**
         * Creates a plain object from a SSL_GeometryData message. Also converts values to other types if specified.
         * @param message SSL_GeometryData
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.SSL_GeometryData, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this SSL_GeometryData to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for SSL_GeometryData
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** SSL_FieldShapeType enum. */
    enum SSL_FieldShapeType {
        Undefined = 0,
        CenterCircle = 1,
        TopTouchLine = 2,
        BottomTouchLine = 3,
        LeftGoalLine = 4,
        RightGoalLine = 5,
        HalfwayLine = 6,
        CenterLine = 7,
        LeftPenaltyStretch = 8,
        RightPenaltyStretch = 9,
        LeftFieldLeftPenaltyStretch = 10,
        LeftFieldRightPenaltyStretch = 11,
        RightFieldLeftPenaltyStretch = 12,
        RightFieldRightPenaltyStretch = 13
    }

    /** Properties of a Referee. */
    interface IReferee {

        /** Referee sourceIdentifier */
        sourceIdentifier?: (string|null);

        /** Referee matchType */
        matchType?: (proto.MatchType|null);

        /** Referee packetTimestamp */
        packetTimestamp: (number|Long);

        /** Referee stage */
        stage: proto.Referee.Stage;

        /** Referee stageTimeLeft */
        stageTimeLeft?: (number|Long|null);

        /** Referee command */
        command: proto.Referee.Command;

        /** Referee commandCounter */
        commandCounter: number;

        /** Referee commandTimestamp */
        commandTimestamp: (number|Long);

        /** Referee yellow */
        yellow: proto.Referee.ITeamInfo;

        /** Referee blue */
        blue: proto.Referee.ITeamInfo;

        /** Referee designatedPosition */
        designatedPosition?: (proto.Referee.IPoint|null);

        /** Referee blueTeamOnPositiveHalf */
        blueTeamOnPositiveHalf?: (boolean|null);

        /** Referee nextCommand */
        nextCommand?: (proto.Referee.Command|null);

        /** Referee gameEvents */
        gameEvents?: (proto.IGameEvent[]|null);

        /** Referee gameEventProposals */
        gameEventProposals?: (proto.IGameEventProposalGroup[]|null);

        /** Referee currentActionTimeRemaining */
        currentActionTimeRemaining?: (number|Long|null);

        /** Referee statusMessage */
        statusMessage?: (string|null);
    }

    /** Represents a Referee. */
    class Referee implements IReferee {

        /**
         * Constructs a new Referee.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IReferee);

        /** Referee sourceIdentifier. */
        public sourceIdentifier: string;

        /** Referee matchType. */
        public matchType: proto.MatchType;

        /** Referee packetTimestamp. */
        public packetTimestamp: (number|Long);

        /** Referee stage. */
        public stage: proto.Referee.Stage;

        /** Referee stageTimeLeft. */
        public stageTimeLeft: (number|Long);

        /** Referee command. */
        public command: proto.Referee.Command;

        /** Referee commandCounter. */
        public commandCounter: number;

        /** Referee commandTimestamp. */
        public commandTimestamp: (number|Long);

        /** Referee yellow. */
        public yellow: proto.Referee.ITeamInfo;

        /** Referee blue. */
        public blue: proto.Referee.ITeamInfo;

        /** Referee designatedPosition. */
        public designatedPosition?: (proto.Referee.IPoint|null);

        /** Referee blueTeamOnPositiveHalf. */
        public blueTeamOnPositiveHalf: boolean;

        /** Referee nextCommand. */
        public nextCommand: proto.Referee.Command;

        /** Referee gameEvents. */
        public gameEvents: proto.IGameEvent[];

        /** Referee gameEventProposals. */
        public gameEventProposals: proto.IGameEventProposalGroup[];

        /** Referee currentActionTimeRemaining. */
        public currentActionTimeRemaining: (number|Long);

        /** Referee statusMessage. */
        public statusMessage: string;

        /**
         * Creates a new Referee instance using the specified properties.
         * @param [properties] Properties to set
         * @returns Referee instance
         */
        public static create(properties?: proto.IReferee): proto.Referee;

        /**
         * Encodes the specified Referee message. Does not implicitly {@link proto.Referee.verify|verify} messages.
         * @param message Referee message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IReferee, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified Referee message, length delimited. Does not implicitly {@link proto.Referee.verify|verify} messages.
         * @param message Referee message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IReferee, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a Referee message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns Referee
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.Referee;

        /**
         * Decodes a Referee message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns Referee
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.Referee;

        /**
         * Verifies a Referee message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a Referee message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns Referee
         */
        public static fromObject(object: { [k: string]: any }): proto.Referee;

        /**
         * Creates a plain object from a Referee message. Also converts values to other types if specified.
         * @param message Referee
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.Referee, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this Referee to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for Referee
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    namespace Referee {

        /** Stage enum. */
        enum Stage {
            NORMAL_FIRST_HALF_PRE = 0,
            NORMAL_FIRST_HALF = 1,
            NORMAL_HALF_TIME = 2,
            NORMAL_SECOND_HALF_PRE = 3,
            NORMAL_SECOND_HALF = 4,
            EXTRA_TIME_BREAK = 5,
            EXTRA_FIRST_HALF_PRE = 6,
            EXTRA_FIRST_HALF = 7,
            EXTRA_HALF_TIME = 8,
            EXTRA_SECOND_HALF_PRE = 9,
            EXTRA_SECOND_HALF = 10,
            PENALTY_SHOOTOUT_BREAK = 11,
            PENALTY_SHOOTOUT = 12,
            POST_GAME = 13
        }

        /** Command enum. */
        enum Command {
            HALT = 0,
            STOP = 1,
            NORMAL_START = 2,
            FORCE_START = 3,
            PREPARE_KICKOFF_YELLOW = 4,
            PREPARE_KICKOFF_BLUE = 5,
            PREPARE_PENALTY_YELLOW = 6,
            PREPARE_PENALTY_BLUE = 7,
            DIRECT_FREE_YELLOW = 8,
            DIRECT_FREE_BLUE = 9,
            TIMEOUT_YELLOW = 12,
            TIMEOUT_BLUE = 13,
            BALL_PLACEMENT_YELLOW = 16,
            BALL_PLACEMENT_BLUE = 17
        }

        /** Properties of a TeamInfo. */
        interface ITeamInfo {

            /** TeamInfo name */
            name: string;

            /** TeamInfo score */
            score: number;

            /** TeamInfo redCards */
            redCards: number;

            /** TeamInfo yellowCardTimes */
            yellowCardTimes?: (number[]|null);

            /** TeamInfo yellowCards */
            yellowCards: number;

            /** TeamInfo timeouts */
            timeouts: number;

            /** TeamInfo timeoutTime */
            timeoutTime: number;

            /** TeamInfo goalkeeper */
            goalkeeper: number;

            /** TeamInfo foulCounter */
            foulCounter?: (number|null);

            /** TeamInfo ballPlacementFailures */
            ballPlacementFailures?: (number|null);

            /** TeamInfo canPlaceBall */
            canPlaceBall?: (boolean|null);

            /** TeamInfo maxAllowedBots */
            maxAllowedBots?: (number|null);

            /** TeamInfo botSubstitutionIntent */
            botSubstitutionIntent?: (boolean|null);

            /** TeamInfo ballPlacementFailuresReached */
            ballPlacementFailuresReached?: (boolean|null);

            /** TeamInfo botSubstitutionAllowed */
            botSubstitutionAllowed?: (boolean|null);
        }

        /** Represents a TeamInfo. */
        class TeamInfo implements ITeamInfo {

            /**
             * Constructs a new TeamInfo.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.Referee.ITeamInfo);

            /** TeamInfo name. */
            public name: string;

            /** TeamInfo score. */
            public score: number;

            /** TeamInfo redCards. */
            public redCards: number;

            /** TeamInfo yellowCardTimes. */
            public yellowCardTimes: number[];

            /** TeamInfo yellowCards. */
            public yellowCards: number;

            /** TeamInfo timeouts. */
            public timeouts: number;

            /** TeamInfo timeoutTime. */
            public timeoutTime: number;

            /** TeamInfo goalkeeper. */
            public goalkeeper: number;

            /** TeamInfo foulCounter. */
            public foulCounter: number;

            /** TeamInfo ballPlacementFailures. */
            public ballPlacementFailures: number;

            /** TeamInfo canPlaceBall. */
            public canPlaceBall: boolean;

            /** TeamInfo maxAllowedBots. */
            public maxAllowedBots: number;

            /** TeamInfo botSubstitutionIntent. */
            public botSubstitutionIntent: boolean;

            /** TeamInfo ballPlacementFailuresReached. */
            public ballPlacementFailuresReached: boolean;

            /** TeamInfo botSubstitutionAllowed. */
            public botSubstitutionAllowed: boolean;

            /**
             * Creates a new TeamInfo instance using the specified properties.
             * @param [properties] Properties to set
             * @returns TeamInfo instance
             */
            public static create(properties?: proto.Referee.ITeamInfo): proto.Referee.TeamInfo;

            /**
             * Encodes the specified TeamInfo message. Does not implicitly {@link proto.Referee.TeamInfo.verify|verify} messages.
             * @param message TeamInfo message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.Referee.ITeamInfo, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified TeamInfo message, length delimited. Does not implicitly {@link proto.Referee.TeamInfo.verify|verify} messages.
             * @param message TeamInfo message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.Referee.ITeamInfo, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a TeamInfo message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns TeamInfo
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.Referee.TeamInfo;

            /**
             * Decodes a TeamInfo message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns TeamInfo
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.Referee.TeamInfo;

            /**
             * Verifies a TeamInfo message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a TeamInfo message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns TeamInfo
             */
            public static fromObject(object: { [k: string]: any }): proto.Referee.TeamInfo;

            /**
             * Creates a plain object from a TeamInfo message. Also converts values to other types if specified.
             * @param message TeamInfo
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.Referee.TeamInfo, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this TeamInfo to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for TeamInfo
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a Point. */
        interface IPoint {

            /** Point x */
            x: number;

            /** Point y */
            y: number;
        }

        /** Represents a Point. */
        class Point implements IPoint {

            /**
             * Constructs a new Point.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.Referee.IPoint);

            /** Point x. */
            public x: number;

            /** Point y. */
            public y: number;

            /**
             * Creates a new Point instance using the specified properties.
             * @param [properties] Properties to set
             * @returns Point instance
             */
            public static create(properties?: proto.Referee.IPoint): proto.Referee.Point;

            /**
             * Encodes the specified Point message. Does not implicitly {@link proto.Referee.Point.verify|verify} messages.
             * @param message Point message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.Referee.IPoint, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified Point message, length delimited. Does not implicitly {@link proto.Referee.Point.verify|verify} messages.
             * @param message Point message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.Referee.IPoint, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a Point message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns Point
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.Referee.Point;

            /**
             * Decodes a Point message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns Point
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.Referee.Point;

            /**
             * Verifies a Point message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a Point message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns Point
             */
            public static fromObject(object: { [k: string]: any }): proto.Referee.Point;

            /**
             * Creates a plain object from a Point message. Also converts values to other types if specified.
             * @param message Point
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.Referee.Point, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this Point to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for Point
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }
    }

    /** Properties of a GameEventProposalGroup. */
    interface IGameEventProposalGroup {

        /** GameEventProposalGroup id */
        id?: (string|null);

        /** GameEventProposalGroup gameEvents */
        gameEvents?: (proto.IGameEvent[]|null);

        /** GameEventProposalGroup accepted */
        accepted?: (boolean|null);
    }

    /** Represents a GameEventProposalGroup. */
    class GameEventProposalGroup implements IGameEventProposalGroup {

        /**
         * Constructs a new GameEventProposalGroup.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IGameEventProposalGroup);

        /** GameEventProposalGroup id. */
        public id: string;

        /** GameEventProposalGroup gameEvents. */
        public gameEvents: proto.IGameEvent[];

        /** GameEventProposalGroup accepted. */
        public accepted: boolean;

        /**
         * Creates a new GameEventProposalGroup instance using the specified properties.
         * @param [properties] Properties to set
         * @returns GameEventProposalGroup instance
         */
        public static create(properties?: proto.IGameEventProposalGroup): proto.GameEventProposalGroup;

        /**
         * Encodes the specified GameEventProposalGroup message. Does not implicitly {@link proto.GameEventProposalGroup.verify|verify} messages.
         * @param message GameEventProposalGroup message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IGameEventProposalGroup, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified GameEventProposalGroup message, length delimited. Does not implicitly {@link proto.GameEventProposalGroup.verify|verify} messages.
         * @param message GameEventProposalGroup message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IGameEventProposalGroup, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a GameEventProposalGroup message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns GameEventProposalGroup
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEventProposalGroup;

        /**
         * Decodes a GameEventProposalGroup message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns GameEventProposalGroup
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEventProposalGroup;

        /**
         * Verifies a GameEventProposalGroup message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a GameEventProposalGroup message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns GameEventProposalGroup
         */
        public static fromObject(object: { [k: string]: any }): proto.GameEventProposalGroup;

        /**
         * Creates a plain object from a GameEventProposalGroup message. Also converts values to other types if specified.
         * @param message GameEventProposalGroup
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.GameEventProposalGroup, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this GameEventProposalGroup to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for GameEventProposalGroup
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** MatchType enum. */
    enum MatchType {
        UNKNOWN_MATCH = 0,
        GROUP_PHASE = 1,
        ELIMINATION_PHASE = 2,
        FRIENDLY = 3
    }

    /** Properties of a GameEvent. */
    interface IGameEvent {

        /** GameEvent id */
        id?: (string|null);

        /** GameEvent type */
        type?: (proto.GameEvent.Type|null);

        /** GameEvent origin */
        origin?: (string[]|null);

        /** GameEvent createdTimestamp */
        createdTimestamp?: (number|Long|null);

        /** GameEvent ballLeftFieldTouchLine */
        ballLeftFieldTouchLine?: (proto.GameEvent.IBallLeftField|null);

        /** GameEvent ballLeftFieldGoalLine */
        ballLeftFieldGoalLine?: (proto.GameEvent.IBallLeftField|null);

        /** GameEvent aimlessKick */
        aimlessKick?: (proto.GameEvent.IAimlessKick|null);

        /** GameEvent attackerTooCloseToDefenseArea */
        attackerTooCloseToDefenseArea?: (proto.GameEvent.IAttackerTooCloseToDefenseArea|null);

        /** GameEvent defenderInDefenseArea */
        defenderInDefenseArea?: (proto.GameEvent.IDefenderInDefenseArea|null);

        /** GameEvent boundaryCrossing */
        boundaryCrossing?: (proto.GameEvent.IBoundaryCrossing|null);

        /** GameEvent keeperHeldBall */
        keeperHeldBall?: (proto.GameEvent.IKeeperHeldBall|null);

        /** GameEvent botDribbledBallTooFar */
        botDribbledBallTooFar?: (proto.GameEvent.IBotDribbledBallTooFar|null);

        /** GameEvent botPushedBot */
        botPushedBot?: (proto.GameEvent.IBotPushedBot|null);

        /** GameEvent botHeldBallDeliberately */
        botHeldBallDeliberately?: (proto.GameEvent.IBotHeldBallDeliberately|null);

        /** GameEvent botTippedOver */
        botTippedOver?: (proto.GameEvent.IBotTippedOver|null);

        /** GameEvent botDroppedParts */
        botDroppedParts?: (proto.GameEvent.IBotDroppedParts|null);

        /** GameEvent attackerTouchedBallInDefenseArea */
        attackerTouchedBallInDefenseArea?: (proto.GameEvent.IAttackerTouchedBallInDefenseArea|null);

        /** GameEvent botKickedBallTooFast */
        botKickedBallTooFast?: (proto.GameEvent.IBotKickedBallTooFast|null);

        /** GameEvent botCrashUnique */
        botCrashUnique?: (proto.GameEvent.IBotCrashUnique|null);

        /** GameEvent botCrashDrawn */
        botCrashDrawn?: (proto.GameEvent.IBotCrashDrawn|null);

        /** GameEvent defenderTooCloseToKickPoint */
        defenderTooCloseToKickPoint?: (proto.GameEvent.IDefenderTooCloseToKickPoint|null);

        /** GameEvent botTooFastInStop */
        botTooFastInStop?: (proto.GameEvent.IBotTooFastInStop|null);

        /** GameEvent botInterferedPlacement */
        botInterferedPlacement?: (proto.GameEvent.IBotInterferedPlacement|null);

        /** GameEvent possibleGoal */
        possibleGoal?: (proto.GameEvent.IGoal|null);

        /** GameEvent goal */
        goal?: (proto.GameEvent.IGoal|null);

        /** GameEvent invalidGoal */
        invalidGoal?: (proto.GameEvent.IGoal|null);

        /** GameEvent attackerDoubleTouchedBall */
        attackerDoubleTouchedBall?: (proto.GameEvent.IAttackerDoubleTouchedBall|null);

        /** GameEvent placementSucceeded */
        placementSucceeded?: (proto.GameEvent.IPlacementSucceeded|null);

        /** GameEvent penaltyKickFailed */
        penaltyKickFailed?: (proto.GameEvent.IPenaltyKickFailed|null);

        /** GameEvent noProgressInGame */
        noProgressInGame?: (proto.GameEvent.INoProgressInGame|null);

        /** GameEvent placementFailed */
        placementFailed?: (proto.GameEvent.IPlacementFailed|null);

        /** GameEvent multipleCards */
        multipleCards?: (proto.GameEvent.IMultipleCards|null);

        /** GameEvent multipleFouls */
        multipleFouls?: (proto.GameEvent.IMultipleFouls|null);

        /** GameEvent botSubstitution */
        botSubstitution?: (proto.GameEvent.IBotSubstitution|null);

        /** GameEvent tooManyRobots */
        tooManyRobots?: (proto.GameEvent.ITooManyRobots|null);

        /** GameEvent challengeFlag */
        challengeFlag?: (proto.GameEvent.IChallengeFlag|null);

        /** GameEvent challengeFlagHandled */
        challengeFlagHandled?: (proto.GameEvent.IChallengeFlagHandled|null);

        /** GameEvent emergencyStop */
        emergencyStop?: (proto.GameEvent.IEmergencyStop|null);

        /** GameEvent unsportingBehaviorMinor */
        unsportingBehaviorMinor?: (proto.GameEvent.IUnsportingBehaviorMinor|null);

        /** GameEvent unsportingBehaviorMajor */
        unsportingBehaviorMajor?: (proto.GameEvent.IUnsportingBehaviorMajor|null);
    }

    /** Represents a GameEvent. */
    class GameEvent implements IGameEvent {

        /**
         * Constructs a new GameEvent.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IGameEvent);

        /** GameEvent id. */
        public id: string;

        /** GameEvent type. */
        public type: proto.GameEvent.Type;

        /** GameEvent origin. */
        public origin: string[];

        /** GameEvent createdTimestamp. */
        public createdTimestamp: (number|Long);

        /** GameEvent ballLeftFieldTouchLine. */
        public ballLeftFieldTouchLine?: (proto.GameEvent.IBallLeftField|null);

        /** GameEvent ballLeftFieldGoalLine. */
        public ballLeftFieldGoalLine?: (proto.GameEvent.IBallLeftField|null);

        /** GameEvent aimlessKick. */
        public aimlessKick?: (proto.GameEvent.IAimlessKick|null);

        /** GameEvent attackerTooCloseToDefenseArea. */
        public attackerTooCloseToDefenseArea?: (proto.GameEvent.IAttackerTooCloseToDefenseArea|null);

        /** GameEvent defenderInDefenseArea. */
        public defenderInDefenseArea?: (proto.GameEvent.IDefenderInDefenseArea|null);

        /** GameEvent boundaryCrossing. */
        public boundaryCrossing?: (proto.GameEvent.IBoundaryCrossing|null);

        /** GameEvent keeperHeldBall. */
        public keeperHeldBall?: (proto.GameEvent.IKeeperHeldBall|null);

        /** GameEvent botDribbledBallTooFar. */
        public botDribbledBallTooFar?: (proto.GameEvent.IBotDribbledBallTooFar|null);

        /** GameEvent botPushedBot. */
        public botPushedBot?: (proto.GameEvent.IBotPushedBot|null);

        /** GameEvent botHeldBallDeliberately. */
        public botHeldBallDeliberately?: (proto.GameEvent.IBotHeldBallDeliberately|null);

        /** GameEvent botTippedOver. */
        public botTippedOver?: (proto.GameEvent.IBotTippedOver|null);

        /** GameEvent botDroppedParts. */
        public botDroppedParts?: (proto.GameEvent.IBotDroppedParts|null);

        /** GameEvent attackerTouchedBallInDefenseArea. */
        public attackerTouchedBallInDefenseArea?: (proto.GameEvent.IAttackerTouchedBallInDefenseArea|null);

        /** GameEvent botKickedBallTooFast. */
        public botKickedBallTooFast?: (proto.GameEvent.IBotKickedBallTooFast|null);

        /** GameEvent botCrashUnique. */
        public botCrashUnique?: (proto.GameEvent.IBotCrashUnique|null);

        /** GameEvent botCrashDrawn. */
        public botCrashDrawn?: (proto.GameEvent.IBotCrashDrawn|null);

        /** GameEvent defenderTooCloseToKickPoint. */
        public defenderTooCloseToKickPoint?: (proto.GameEvent.IDefenderTooCloseToKickPoint|null);

        /** GameEvent botTooFastInStop. */
        public botTooFastInStop?: (proto.GameEvent.IBotTooFastInStop|null);

        /** GameEvent botInterferedPlacement. */
        public botInterferedPlacement?: (proto.GameEvent.IBotInterferedPlacement|null);

        /** GameEvent possibleGoal. */
        public possibleGoal?: (proto.GameEvent.IGoal|null);

        /** GameEvent goal. */
        public goal?: (proto.GameEvent.IGoal|null);

        /** GameEvent invalidGoal. */
        public invalidGoal?: (proto.GameEvent.IGoal|null);

        /** GameEvent attackerDoubleTouchedBall. */
        public attackerDoubleTouchedBall?: (proto.GameEvent.IAttackerDoubleTouchedBall|null);

        /** GameEvent placementSucceeded. */
        public placementSucceeded?: (proto.GameEvent.IPlacementSucceeded|null);

        /** GameEvent penaltyKickFailed. */
        public penaltyKickFailed?: (proto.GameEvent.IPenaltyKickFailed|null);

        /** GameEvent noProgressInGame. */
        public noProgressInGame?: (proto.GameEvent.INoProgressInGame|null);

        /** GameEvent placementFailed. */
        public placementFailed?: (proto.GameEvent.IPlacementFailed|null);

        /** GameEvent multipleCards. */
        public multipleCards?: (proto.GameEvent.IMultipleCards|null);

        /** GameEvent multipleFouls. */
        public multipleFouls?: (proto.GameEvent.IMultipleFouls|null);

        /** GameEvent botSubstitution. */
        public botSubstitution?: (proto.GameEvent.IBotSubstitution|null);

        /** GameEvent tooManyRobots. */
        public tooManyRobots?: (proto.GameEvent.ITooManyRobots|null);

        /** GameEvent challengeFlag. */
        public challengeFlag?: (proto.GameEvent.IChallengeFlag|null);

        /** GameEvent challengeFlagHandled. */
        public challengeFlagHandled?: (proto.GameEvent.IChallengeFlagHandled|null);

        /** GameEvent emergencyStop. */
        public emergencyStop?: (proto.GameEvent.IEmergencyStop|null);

        /** GameEvent unsportingBehaviorMinor. */
        public unsportingBehaviorMinor?: (proto.GameEvent.IUnsportingBehaviorMinor|null);

        /** GameEvent unsportingBehaviorMajor. */
        public unsportingBehaviorMajor?: (proto.GameEvent.IUnsportingBehaviorMajor|null);

        /** GameEvent event. */
        public event?: ("ballLeftFieldTouchLine"|"ballLeftFieldGoalLine"|"aimlessKick"|"attackerTooCloseToDefenseArea"|"defenderInDefenseArea"|"boundaryCrossing"|"keeperHeldBall"|"botDribbledBallTooFar"|"botPushedBot"|"botHeldBallDeliberately"|"botTippedOver"|"botDroppedParts"|"attackerTouchedBallInDefenseArea"|"botKickedBallTooFast"|"botCrashUnique"|"botCrashDrawn"|"defenderTooCloseToKickPoint"|"botTooFastInStop"|"botInterferedPlacement"|"possibleGoal"|"goal"|"invalidGoal"|"attackerDoubleTouchedBall"|"placementSucceeded"|"penaltyKickFailed"|"noProgressInGame"|"placementFailed"|"multipleCards"|"multipleFouls"|"botSubstitution"|"tooManyRobots"|"challengeFlag"|"challengeFlagHandled"|"emergencyStop"|"unsportingBehaviorMinor"|"unsportingBehaviorMajor");

        /**
         * Creates a new GameEvent instance using the specified properties.
         * @param [properties] Properties to set
         * @returns GameEvent instance
         */
        public static create(properties?: proto.IGameEvent): proto.GameEvent;

        /**
         * Encodes the specified GameEvent message. Does not implicitly {@link proto.GameEvent.verify|verify} messages.
         * @param message GameEvent message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IGameEvent, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified GameEvent message, length delimited. Does not implicitly {@link proto.GameEvent.verify|verify} messages.
         * @param message GameEvent message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IGameEvent, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a GameEvent message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns GameEvent
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent;

        /**
         * Decodes a GameEvent message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns GameEvent
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent;

        /**
         * Verifies a GameEvent message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a GameEvent message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns GameEvent
         */
        public static fromObject(object: { [k: string]: any }): proto.GameEvent;

        /**
         * Creates a plain object from a GameEvent message. Also converts values to other types if specified.
         * @param message GameEvent
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.GameEvent, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this GameEvent to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for GameEvent
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    namespace GameEvent {

        /** Properties of a BallLeftField. */
        interface IBallLeftField {

            /** BallLeftField byTeam */
            byTeam: proto.Team;

            /** BallLeftField byBot */
            byBot?: (number|null);

            /** BallLeftField location */
            location?: (proto.IVector2|null);
        }

        /** Represents a BallLeftField. */
        class BallLeftField implements IBallLeftField {

            /**
             * Constructs a new BallLeftField.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IBallLeftField);

            /** BallLeftField byTeam. */
            public byTeam: proto.Team;

            /** BallLeftField byBot. */
            public byBot: number;

            /** BallLeftField location. */
            public location?: (proto.IVector2|null);

            /**
             * Creates a new BallLeftField instance using the specified properties.
             * @param [properties] Properties to set
             * @returns BallLeftField instance
             */
            public static create(properties?: proto.GameEvent.IBallLeftField): proto.GameEvent.BallLeftField;

            /**
             * Encodes the specified BallLeftField message. Does not implicitly {@link proto.GameEvent.BallLeftField.verify|verify} messages.
             * @param message BallLeftField message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IBallLeftField, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified BallLeftField message, length delimited. Does not implicitly {@link proto.GameEvent.BallLeftField.verify|verify} messages.
             * @param message BallLeftField message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IBallLeftField, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a BallLeftField message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns BallLeftField
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.BallLeftField;

            /**
             * Decodes a BallLeftField message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns BallLeftField
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.BallLeftField;

            /**
             * Verifies a BallLeftField message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a BallLeftField message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns BallLeftField
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.BallLeftField;

            /**
             * Creates a plain object from a BallLeftField message. Also converts values to other types if specified.
             * @param message BallLeftField
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.BallLeftField, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this BallLeftField to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for BallLeftField
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of an AimlessKick. */
        interface IAimlessKick {

            /** AimlessKick byTeam */
            byTeam: proto.Team;

            /** AimlessKick byBot */
            byBot?: (number|null);

            /** AimlessKick location */
            location?: (proto.IVector2|null);

            /** AimlessKick kickLocation */
            kickLocation?: (proto.IVector2|null);
        }

        /** Represents an AimlessKick. */
        class AimlessKick implements IAimlessKick {

            /**
             * Constructs a new AimlessKick.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IAimlessKick);

            /** AimlessKick byTeam. */
            public byTeam: proto.Team;

            /** AimlessKick byBot. */
            public byBot: number;

            /** AimlessKick location. */
            public location?: (proto.IVector2|null);

            /** AimlessKick kickLocation. */
            public kickLocation?: (proto.IVector2|null);

            /**
             * Creates a new AimlessKick instance using the specified properties.
             * @param [properties] Properties to set
             * @returns AimlessKick instance
             */
            public static create(properties?: proto.GameEvent.IAimlessKick): proto.GameEvent.AimlessKick;

            /**
             * Encodes the specified AimlessKick message. Does not implicitly {@link proto.GameEvent.AimlessKick.verify|verify} messages.
             * @param message AimlessKick message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IAimlessKick, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified AimlessKick message, length delimited. Does not implicitly {@link proto.GameEvent.AimlessKick.verify|verify} messages.
             * @param message AimlessKick message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IAimlessKick, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes an AimlessKick message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns AimlessKick
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.AimlessKick;

            /**
             * Decodes an AimlessKick message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns AimlessKick
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.AimlessKick;

            /**
             * Verifies an AimlessKick message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates an AimlessKick message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns AimlessKick
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.AimlessKick;

            /**
             * Creates a plain object from an AimlessKick message. Also converts values to other types if specified.
             * @param message AimlessKick
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.AimlessKick, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this AimlessKick to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for AimlessKick
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a Goal. */
        interface IGoal {

            /** Goal byTeam */
            byTeam: proto.Team;

            /** Goal kickingTeam */
            kickingTeam?: (proto.Team|null);

            /** Goal kickingBot */
            kickingBot?: (number|null);

            /** Goal location */
            location?: (proto.IVector2|null);

            /** Goal kickLocation */
            kickLocation?: (proto.IVector2|null);

            /** Goal maxBallHeight */
            maxBallHeight?: (number|null);

            /** Goal numRobotsByTeam */
            numRobotsByTeam?: (number|null);

            /** Goal lastTouchByTeam */
            lastTouchByTeam?: (number|Long|null);

            /** Goal message */
            message?: (string|null);
        }

        /** Represents a Goal. */
        class Goal implements IGoal {

            /**
             * Constructs a new Goal.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IGoal);

            /** Goal byTeam. */
            public byTeam: proto.Team;

            /** Goal kickingTeam. */
            public kickingTeam: proto.Team;

            /** Goal kickingBot. */
            public kickingBot: number;

            /** Goal location. */
            public location?: (proto.IVector2|null);

            /** Goal kickLocation. */
            public kickLocation?: (proto.IVector2|null);

            /** Goal maxBallHeight. */
            public maxBallHeight: number;

            /** Goal numRobotsByTeam. */
            public numRobotsByTeam: number;

            /** Goal lastTouchByTeam. */
            public lastTouchByTeam: (number|Long);

            /** Goal message. */
            public message: string;

            /**
             * Creates a new Goal instance using the specified properties.
             * @param [properties] Properties to set
             * @returns Goal instance
             */
            public static create(properties?: proto.GameEvent.IGoal): proto.GameEvent.Goal;

            /**
             * Encodes the specified Goal message. Does not implicitly {@link proto.GameEvent.Goal.verify|verify} messages.
             * @param message Goal message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IGoal, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified Goal message, length delimited. Does not implicitly {@link proto.GameEvent.Goal.verify|verify} messages.
             * @param message Goal message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IGoal, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a Goal message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns Goal
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.Goal;

            /**
             * Decodes a Goal message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns Goal
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.Goal;

            /**
             * Verifies a Goal message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a Goal message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns Goal
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.Goal;

            /**
             * Creates a plain object from a Goal message. Also converts values to other types if specified.
             * @param message Goal
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.Goal, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this Goal to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for Goal
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of an IndirectGoal. */
        interface IIndirectGoal {

            /** IndirectGoal byTeam */
            byTeam: proto.Team;

            /** IndirectGoal byBot */
            byBot?: (number|null);

            /** IndirectGoal location */
            location?: (proto.IVector2|null);

            /** IndirectGoal kickLocation */
            kickLocation?: (proto.IVector2|null);
        }

        /** Represents an IndirectGoal. */
        class IndirectGoal implements IIndirectGoal {

            /**
             * Constructs a new IndirectGoal.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IIndirectGoal);

            /** IndirectGoal byTeam. */
            public byTeam: proto.Team;

            /** IndirectGoal byBot. */
            public byBot: number;

            /** IndirectGoal location. */
            public location?: (proto.IVector2|null);

            /** IndirectGoal kickLocation. */
            public kickLocation?: (proto.IVector2|null);

            /**
             * Creates a new IndirectGoal instance using the specified properties.
             * @param [properties] Properties to set
             * @returns IndirectGoal instance
             */
            public static create(properties?: proto.GameEvent.IIndirectGoal): proto.GameEvent.IndirectGoal;

            /**
             * Encodes the specified IndirectGoal message. Does not implicitly {@link proto.GameEvent.IndirectGoal.verify|verify} messages.
             * @param message IndirectGoal message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IIndirectGoal, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified IndirectGoal message, length delimited. Does not implicitly {@link proto.GameEvent.IndirectGoal.verify|verify} messages.
             * @param message IndirectGoal message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IIndirectGoal, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes an IndirectGoal message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns IndirectGoal
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.IndirectGoal;

            /**
             * Decodes an IndirectGoal message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns IndirectGoal
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.IndirectGoal;

            /**
             * Verifies an IndirectGoal message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates an IndirectGoal message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns IndirectGoal
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.IndirectGoal;

            /**
             * Creates a plain object from an IndirectGoal message. Also converts values to other types if specified.
             * @param message IndirectGoal
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.IndirectGoal, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this IndirectGoal to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for IndirectGoal
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a ChippedGoal. */
        interface IChippedGoal {

            /** ChippedGoal byTeam */
            byTeam: proto.Team;

            /** ChippedGoal byBot */
            byBot?: (number|null);

            /** ChippedGoal location */
            location?: (proto.IVector2|null);

            /** ChippedGoal kickLocation */
            kickLocation?: (proto.IVector2|null);

            /** ChippedGoal maxBallHeight */
            maxBallHeight?: (number|null);
        }

        /** Represents a ChippedGoal. */
        class ChippedGoal implements IChippedGoal {

            /**
             * Constructs a new ChippedGoal.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IChippedGoal);

            /** ChippedGoal byTeam. */
            public byTeam: proto.Team;

            /** ChippedGoal byBot. */
            public byBot: number;

            /** ChippedGoal location. */
            public location?: (proto.IVector2|null);

            /** ChippedGoal kickLocation. */
            public kickLocation?: (proto.IVector2|null);

            /** ChippedGoal maxBallHeight. */
            public maxBallHeight: number;

            /**
             * Creates a new ChippedGoal instance using the specified properties.
             * @param [properties] Properties to set
             * @returns ChippedGoal instance
             */
            public static create(properties?: proto.GameEvent.IChippedGoal): proto.GameEvent.ChippedGoal;

            /**
             * Encodes the specified ChippedGoal message. Does not implicitly {@link proto.GameEvent.ChippedGoal.verify|verify} messages.
             * @param message ChippedGoal message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IChippedGoal, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified ChippedGoal message, length delimited. Does not implicitly {@link proto.GameEvent.ChippedGoal.verify|verify} messages.
             * @param message ChippedGoal message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IChippedGoal, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a ChippedGoal message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns ChippedGoal
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.ChippedGoal;

            /**
             * Decodes a ChippedGoal message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns ChippedGoal
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.ChippedGoal;

            /**
             * Verifies a ChippedGoal message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a ChippedGoal message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns ChippedGoal
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.ChippedGoal;

            /**
             * Creates a plain object from a ChippedGoal message. Also converts values to other types if specified.
             * @param message ChippedGoal
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.ChippedGoal, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this ChippedGoal to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for ChippedGoal
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a BotTooFastInStop. */
        interface IBotTooFastInStop {

            /** BotTooFastInStop byTeam */
            byTeam: proto.Team;

            /** BotTooFastInStop byBot */
            byBot?: (number|null);

            /** BotTooFastInStop location */
            location?: (proto.IVector2|null);

            /** BotTooFastInStop speed */
            speed?: (number|null);
        }

        /** Represents a BotTooFastInStop. */
        class BotTooFastInStop implements IBotTooFastInStop {

            /**
             * Constructs a new BotTooFastInStop.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IBotTooFastInStop);

            /** BotTooFastInStop byTeam. */
            public byTeam: proto.Team;

            /** BotTooFastInStop byBot. */
            public byBot: number;

            /** BotTooFastInStop location. */
            public location?: (proto.IVector2|null);

            /** BotTooFastInStop speed. */
            public speed: number;

            /**
             * Creates a new BotTooFastInStop instance using the specified properties.
             * @param [properties] Properties to set
             * @returns BotTooFastInStop instance
             */
            public static create(properties?: proto.GameEvent.IBotTooFastInStop): proto.GameEvent.BotTooFastInStop;

            /**
             * Encodes the specified BotTooFastInStop message. Does not implicitly {@link proto.GameEvent.BotTooFastInStop.verify|verify} messages.
             * @param message BotTooFastInStop message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IBotTooFastInStop, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified BotTooFastInStop message, length delimited. Does not implicitly {@link proto.GameEvent.BotTooFastInStop.verify|verify} messages.
             * @param message BotTooFastInStop message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IBotTooFastInStop, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a BotTooFastInStop message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns BotTooFastInStop
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.BotTooFastInStop;

            /**
             * Decodes a BotTooFastInStop message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns BotTooFastInStop
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.BotTooFastInStop;

            /**
             * Verifies a BotTooFastInStop message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a BotTooFastInStop message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns BotTooFastInStop
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.BotTooFastInStop;

            /**
             * Creates a plain object from a BotTooFastInStop message. Also converts values to other types if specified.
             * @param message BotTooFastInStop
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.BotTooFastInStop, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this BotTooFastInStop to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for BotTooFastInStop
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a DefenderTooCloseToKickPoint. */
        interface IDefenderTooCloseToKickPoint {

            /** DefenderTooCloseToKickPoint byTeam */
            byTeam: proto.Team;

            /** DefenderTooCloseToKickPoint byBot */
            byBot?: (number|null);

            /** DefenderTooCloseToKickPoint location */
            location?: (proto.IVector2|null);

            /** DefenderTooCloseToKickPoint distance */
            distance?: (number|null);
        }

        /** Represents a DefenderTooCloseToKickPoint. */
        class DefenderTooCloseToKickPoint implements IDefenderTooCloseToKickPoint {

            /**
             * Constructs a new DefenderTooCloseToKickPoint.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IDefenderTooCloseToKickPoint);

            /** DefenderTooCloseToKickPoint byTeam. */
            public byTeam: proto.Team;

            /** DefenderTooCloseToKickPoint byBot. */
            public byBot: number;

            /** DefenderTooCloseToKickPoint location. */
            public location?: (proto.IVector2|null);

            /** DefenderTooCloseToKickPoint distance. */
            public distance: number;

            /**
             * Creates a new DefenderTooCloseToKickPoint instance using the specified properties.
             * @param [properties] Properties to set
             * @returns DefenderTooCloseToKickPoint instance
             */
            public static create(properties?: proto.GameEvent.IDefenderTooCloseToKickPoint): proto.GameEvent.DefenderTooCloseToKickPoint;

            /**
             * Encodes the specified DefenderTooCloseToKickPoint message. Does not implicitly {@link proto.GameEvent.DefenderTooCloseToKickPoint.verify|verify} messages.
             * @param message DefenderTooCloseToKickPoint message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IDefenderTooCloseToKickPoint, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified DefenderTooCloseToKickPoint message, length delimited. Does not implicitly {@link proto.GameEvent.DefenderTooCloseToKickPoint.verify|verify} messages.
             * @param message DefenderTooCloseToKickPoint message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IDefenderTooCloseToKickPoint, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a DefenderTooCloseToKickPoint message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns DefenderTooCloseToKickPoint
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.DefenderTooCloseToKickPoint;

            /**
             * Decodes a DefenderTooCloseToKickPoint message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns DefenderTooCloseToKickPoint
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.DefenderTooCloseToKickPoint;

            /**
             * Verifies a DefenderTooCloseToKickPoint message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a DefenderTooCloseToKickPoint message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns DefenderTooCloseToKickPoint
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.DefenderTooCloseToKickPoint;

            /**
             * Creates a plain object from a DefenderTooCloseToKickPoint message. Also converts values to other types if specified.
             * @param message DefenderTooCloseToKickPoint
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.DefenderTooCloseToKickPoint, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this DefenderTooCloseToKickPoint to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for DefenderTooCloseToKickPoint
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a BotCrashDrawn. */
        interface IBotCrashDrawn {

            /** BotCrashDrawn botYellow */
            botYellow?: (number|null);

            /** BotCrashDrawn botBlue */
            botBlue?: (number|null);

            /** BotCrashDrawn location */
            location?: (proto.IVector2|null);

            /** BotCrashDrawn crashSpeed */
            crashSpeed?: (number|null);

            /** BotCrashDrawn speedDiff */
            speedDiff?: (number|null);

            /** BotCrashDrawn crashAngle */
            crashAngle?: (number|null);
        }

        /** Represents a BotCrashDrawn. */
        class BotCrashDrawn implements IBotCrashDrawn {

            /**
             * Constructs a new BotCrashDrawn.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IBotCrashDrawn);

            /** BotCrashDrawn botYellow. */
            public botYellow: number;

            /** BotCrashDrawn botBlue. */
            public botBlue: number;

            /** BotCrashDrawn location. */
            public location?: (proto.IVector2|null);

            /** BotCrashDrawn crashSpeed. */
            public crashSpeed: number;

            /** BotCrashDrawn speedDiff. */
            public speedDiff: number;

            /** BotCrashDrawn crashAngle. */
            public crashAngle: number;

            /**
             * Creates a new BotCrashDrawn instance using the specified properties.
             * @param [properties] Properties to set
             * @returns BotCrashDrawn instance
             */
            public static create(properties?: proto.GameEvent.IBotCrashDrawn): proto.GameEvent.BotCrashDrawn;

            /**
             * Encodes the specified BotCrashDrawn message. Does not implicitly {@link proto.GameEvent.BotCrashDrawn.verify|verify} messages.
             * @param message BotCrashDrawn message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IBotCrashDrawn, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified BotCrashDrawn message, length delimited. Does not implicitly {@link proto.GameEvent.BotCrashDrawn.verify|verify} messages.
             * @param message BotCrashDrawn message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IBotCrashDrawn, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a BotCrashDrawn message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns BotCrashDrawn
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.BotCrashDrawn;

            /**
             * Decodes a BotCrashDrawn message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns BotCrashDrawn
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.BotCrashDrawn;

            /**
             * Verifies a BotCrashDrawn message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a BotCrashDrawn message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns BotCrashDrawn
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.BotCrashDrawn;

            /**
             * Creates a plain object from a BotCrashDrawn message. Also converts values to other types if specified.
             * @param message BotCrashDrawn
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.BotCrashDrawn, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this BotCrashDrawn to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for BotCrashDrawn
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a BotCrashUnique. */
        interface IBotCrashUnique {

            /** BotCrashUnique byTeam */
            byTeam: proto.Team;

            /** BotCrashUnique violator */
            violator?: (number|null);

            /** BotCrashUnique victim */
            victim?: (number|null);

            /** BotCrashUnique location */
            location?: (proto.IVector2|null);

            /** BotCrashUnique crashSpeed */
            crashSpeed?: (number|null);

            /** BotCrashUnique speedDiff */
            speedDiff?: (number|null);

            /** BotCrashUnique crashAngle */
            crashAngle?: (number|null);
        }

        /** Represents a BotCrashUnique. */
        class BotCrashUnique implements IBotCrashUnique {

            /**
             * Constructs a new BotCrashUnique.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IBotCrashUnique);

            /** BotCrashUnique byTeam. */
            public byTeam: proto.Team;

            /** BotCrashUnique violator. */
            public violator: number;

            /** BotCrashUnique victim. */
            public victim: number;

            /** BotCrashUnique location. */
            public location?: (proto.IVector2|null);

            /** BotCrashUnique crashSpeed. */
            public crashSpeed: number;

            /** BotCrashUnique speedDiff. */
            public speedDiff: number;

            /** BotCrashUnique crashAngle. */
            public crashAngle: number;

            /**
             * Creates a new BotCrashUnique instance using the specified properties.
             * @param [properties] Properties to set
             * @returns BotCrashUnique instance
             */
            public static create(properties?: proto.GameEvent.IBotCrashUnique): proto.GameEvent.BotCrashUnique;

            /**
             * Encodes the specified BotCrashUnique message. Does not implicitly {@link proto.GameEvent.BotCrashUnique.verify|verify} messages.
             * @param message BotCrashUnique message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IBotCrashUnique, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified BotCrashUnique message, length delimited. Does not implicitly {@link proto.GameEvent.BotCrashUnique.verify|verify} messages.
             * @param message BotCrashUnique message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IBotCrashUnique, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a BotCrashUnique message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns BotCrashUnique
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.BotCrashUnique;

            /**
             * Decodes a BotCrashUnique message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns BotCrashUnique
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.BotCrashUnique;

            /**
             * Verifies a BotCrashUnique message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a BotCrashUnique message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns BotCrashUnique
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.BotCrashUnique;

            /**
             * Creates a plain object from a BotCrashUnique message. Also converts values to other types if specified.
             * @param message BotCrashUnique
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.BotCrashUnique, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this BotCrashUnique to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for BotCrashUnique
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a BotPushedBot. */
        interface IBotPushedBot {

            /** BotPushedBot byTeam */
            byTeam: proto.Team;

            /** BotPushedBot violator */
            violator?: (number|null);

            /** BotPushedBot victim */
            victim?: (number|null);

            /** BotPushedBot location */
            location?: (proto.IVector2|null);

            /** BotPushedBot pushedDistance */
            pushedDistance?: (number|null);
        }

        /** Represents a BotPushedBot. */
        class BotPushedBot implements IBotPushedBot {

            /**
             * Constructs a new BotPushedBot.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IBotPushedBot);

            /** BotPushedBot byTeam. */
            public byTeam: proto.Team;

            /** BotPushedBot violator. */
            public violator: number;

            /** BotPushedBot victim. */
            public victim: number;

            /** BotPushedBot location. */
            public location?: (proto.IVector2|null);

            /** BotPushedBot pushedDistance. */
            public pushedDistance: number;

            /**
             * Creates a new BotPushedBot instance using the specified properties.
             * @param [properties] Properties to set
             * @returns BotPushedBot instance
             */
            public static create(properties?: proto.GameEvent.IBotPushedBot): proto.GameEvent.BotPushedBot;

            /**
             * Encodes the specified BotPushedBot message. Does not implicitly {@link proto.GameEvent.BotPushedBot.verify|verify} messages.
             * @param message BotPushedBot message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IBotPushedBot, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified BotPushedBot message, length delimited. Does not implicitly {@link proto.GameEvent.BotPushedBot.verify|verify} messages.
             * @param message BotPushedBot message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IBotPushedBot, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a BotPushedBot message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns BotPushedBot
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.BotPushedBot;

            /**
             * Decodes a BotPushedBot message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns BotPushedBot
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.BotPushedBot;

            /**
             * Verifies a BotPushedBot message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a BotPushedBot message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns BotPushedBot
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.BotPushedBot;

            /**
             * Creates a plain object from a BotPushedBot message. Also converts values to other types if specified.
             * @param message BotPushedBot
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.BotPushedBot, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this BotPushedBot to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for BotPushedBot
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a BotTippedOver. */
        interface IBotTippedOver {

            /** BotTippedOver byTeam */
            byTeam: proto.Team;

            /** BotTippedOver byBot */
            byBot?: (number|null);

            /** BotTippedOver location */
            location?: (proto.IVector2|null);

            /** BotTippedOver ballLocation */
            ballLocation?: (proto.IVector2|null);
        }

        /** Represents a BotTippedOver. */
        class BotTippedOver implements IBotTippedOver {

            /**
             * Constructs a new BotTippedOver.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IBotTippedOver);

            /** BotTippedOver byTeam. */
            public byTeam: proto.Team;

            /** BotTippedOver byBot. */
            public byBot: number;

            /** BotTippedOver location. */
            public location?: (proto.IVector2|null);

            /** BotTippedOver ballLocation. */
            public ballLocation?: (proto.IVector2|null);

            /**
             * Creates a new BotTippedOver instance using the specified properties.
             * @param [properties] Properties to set
             * @returns BotTippedOver instance
             */
            public static create(properties?: proto.GameEvent.IBotTippedOver): proto.GameEvent.BotTippedOver;

            /**
             * Encodes the specified BotTippedOver message. Does not implicitly {@link proto.GameEvent.BotTippedOver.verify|verify} messages.
             * @param message BotTippedOver message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IBotTippedOver, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified BotTippedOver message, length delimited. Does not implicitly {@link proto.GameEvent.BotTippedOver.verify|verify} messages.
             * @param message BotTippedOver message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IBotTippedOver, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a BotTippedOver message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns BotTippedOver
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.BotTippedOver;

            /**
             * Decodes a BotTippedOver message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns BotTippedOver
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.BotTippedOver;

            /**
             * Verifies a BotTippedOver message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a BotTippedOver message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns BotTippedOver
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.BotTippedOver;

            /**
             * Creates a plain object from a BotTippedOver message. Also converts values to other types if specified.
             * @param message BotTippedOver
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.BotTippedOver, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this BotTippedOver to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for BotTippedOver
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a BotDroppedParts. */
        interface IBotDroppedParts {

            /** BotDroppedParts byTeam */
            byTeam: proto.Team;

            /** BotDroppedParts byBot */
            byBot?: (number|null);

            /** BotDroppedParts location */
            location?: (proto.IVector2|null);

            /** BotDroppedParts ballLocation */
            ballLocation?: (proto.IVector2|null);
        }

        /** Represents a BotDroppedParts. */
        class BotDroppedParts implements IBotDroppedParts {

            /**
             * Constructs a new BotDroppedParts.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IBotDroppedParts);

            /** BotDroppedParts byTeam. */
            public byTeam: proto.Team;

            /** BotDroppedParts byBot. */
            public byBot: number;

            /** BotDroppedParts location. */
            public location?: (proto.IVector2|null);

            /** BotDroppedParts ballLocation. */
            public ballLocation?: (proto.IVector2|null);

            /**
             * Creates a new BotDroppedParts instance using the specified properties.
             * @param [properties] Properties to set
             * @returns BotDroppedParts instance
             */
            public static create(properties?: proto.GameEvent.IBotDroppedParts): proto.GameEvent.BotDroppedParts;

            /**
             * Encodes the specified BotDroppedParts message. Does not implicitly {@link proto.GameEvent.BotDroppedParts.verify|verify} messages.
             * @param message BotDroppedParts message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IBotDroppedParts, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified BotDroppedParts message, length delimited. Does not implicitly {@link proto.GameEvent.BotDroppedParts.verify|verify} messages.
             * @param message BotDroppedParts message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IBotDroppedParts, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a BotDroppedParts message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns BotDroppedParts
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.BotDroppedParts;

            /**
             * Decodes a BotDroppedParts message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns BotDroppedParts
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.BotDroppedParts;

            /**
             * Verifies a BotDroppedParts message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a BotDroppedParts message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns BotDroppedParts
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.BotDroppedParts;

            /**
             * Creates a plain object from a BotDroppedParts message. Also converts values to other types if specified.
             * @param message BotDroppedParts
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.BotDroppedParts, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this BotDroppedParts to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for BotDroppedParts
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a DefenderInDefenseArea. */
        interface IDefenderInDefenseArea {

            /** DefenderInDefenseArea byTeam */
            byTeam: proto.Team;

            /** DefenderInDefenseArea byBot */
            byBot?: (number|null);

            /** DefenderInDefenseArea location */
            location?: (proto.IVector2|null);

            /** DefenderInDefenseArea distance */
            distance?: (number|null);
        }

        /** Represents a DefenderInDefenseArea. */
        class DefenderInDefenseArea implements IDefenderInDefenseArea {

            /**
             * Constructs a new DefenderInDefenseArea.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IDefenderInDefenseArea);

            /** DefenderInDefenseArea byTeam. */
            public byTeam: proto.Team;

            /** DefenderInDefenseArea byBot. */
            public byBot: number;

            /** DefenderInDefenseArea location. */
            public location?: (proto.IVector2|null);

            /** DefenderInDefenseArea distance. */
            public distance: number;

            /**
             * Creates a new DefenderInDefenseArea instance using the specified properties.
             * @param [properties] Properties to set
             * @returns DefenderInDefenseArea instance
             */
            public static create(properties?: proto.GameEvent.IDefenderInDefenseArea): proto.GameEvent.DefenderInDefenseArea;

            /**
             * Encodes the specified DefenderInDefenseArea message. Does not implicitly {@link proto.GameEvent.DefenderInDefenseArea.verify|verify} messages.
             * @param message DefenderInDefenseArea message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IDefenderInDefenseArea, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified DefenderInDefenseArea message, length delimited. Does not implicitly {@link proto.GameEvent.DefenderInDefenseArea.verify|verify} messages.
             * @param message DefenderInDefenseArea message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IDefenderInDefenseArea, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a DefenderInDefenseArea message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns DefenderInDefenseArea
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.DefenderInDefenseArea;

            /**
             * Decodes a DefenderInDefenseArea message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns DefenderInDefenseArea
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.DefenderInDefenseArea;

            /**
             * Verifies a DefenderInDefenseArea message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a DefenderInDefenseArea message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns DefenderInDefenseArea
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.DefenderInDefenseArea;

            /**
             * Creates a plain object from a DefenderInDefenseArea message. Also converts values to other types if specified.
             * @param message DefenderInDefenseArea
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.DefenderInDefenseArea, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this DefenderInDefenseArea to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for DefenderInDefenseArea
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a DefenderInDefenseAreaPartially. */
        interface IDefenderInDefenseAreaPartially {

            /** DefenderInDefenseAreaPartially byTeam */
            byTeam: proto.Team;

            /** DefenderInDefenseAreaPartially byBot */
            byBot?: (number|null);

            /** DefenderInDefenseAreaPartially location */
            location?: (proto.IVector2|null);

            /** DefenderInDefenseAreaPartially distance */
            distance?: (number|null);

            /** DefenderInDefenseAreaPartially ballLocation */
            ballLocation?: (proto.IVector2|null);
        }

        /** Represents a DefenderInDefenseAreaPartially. */
        class DefenderInDefenseAreaPartially implements IDefenderInDefenseAreaPartially {

            /**
             * Constructs a new DefenderInDefenseAreaPartially.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IDefenderInDefenseAreaPartially);

            /** DefenderInDefenseAreaPartially byTeam. */
            public byTeam: proto.Team;

            /** DefenderInDefenseAreaPartially byBot. */
            public byBot: number;

            /** DefenderInDefenseAreaPartially location. */
            public location?: (proto.IVector2|null);

            /** DefenderInDefenseAreaPartially distance. */
            public distance: number;

            /** DefenderInDefenseAreaPartially ballLocation. */
            public ballLocation?: (proto.IVector2|null);

            /**
             * Creates a new DefenderInDefenseAreaPartially instance using the specified properties.
             * @param [properties] Properties to set
             * @returns DefenderInDefenseAreaPartially instance
             */
            public static create(properties?: proto.GameEvent.IDefenderInDefenseAreaPartially): proto.GameEvent.DefenderInDefenseAreaPartially;

            /**
             * Encodes the specified DefenderInDefenseAreaPartially message. Does not implicitly {@link proto.GameEvent.DefenderInDefenseAreaPartially.verify|verify} messages.
             * @param message DefenderInDefenseAreaPartially message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IDefenderInDefenseAreaPartially, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified DefenderInDefenseAreaPartially message, length delimited. Does not implicitly {@link proto.GameEvent.DefenderInDefenseAreaPartially.verify|verify} messages.
             * @param message DefenderInDefenseAreaPartially message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IDefenderInDefenseAreaPartially, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a DefenderInDefenseAreaPartially message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns DefenderInDefenseAreaPartially
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.DefenderInDefenseAreaPartially;

            /**
             * Decodes a DefenderInDefenseAreaPartially message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns DefenderInDefenseAreaPartially
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.DefenderInDefenseAreaPartially;

            /**
             * Verifies a DefenderInDefenseAreaPartially message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a DefenderInDefenseAreaPartially message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns DefenderInDefenseAreaPartially
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.DefenderInDefenseAreaPartially;

            /**
             * Creates a plain object from a DefenderInDefenseAreaPartially message. Also converts values to other types if specified.
             * @param message DefenderInDefenseAreaPartially
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.DefenderInDefenseAreaPartially, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this DefenderInDefenseAreaPartially to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for DefenderInDefenseAreaPartially
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of an AttackerTouchedBallInDefenseArea. */
        interface IAttackerTouchedBallInDefenseArea {

            /** AttackerTouchedBallInDefenseArea byTeam */
            byTeam: proto.Team;

            /** AttackerTouchedBallInDefenseArea byBot */
            byBot?: (number|null);

            /** AttackerTouchedBallInDefenseArea location */
            location?: (proto.IVector2|null);

            /** AttackerTouchedBallInDefenseArea distance */
            distance?: (number|null);
        }

        /** Represents an AttackerTouchedBallInDefenseArea. */
        class AttackerTouchedBallInDefenseArea implements IAttackerTouchedBallInDefenseArea {

            /**
             * Constructs a new AttackerTouchedBallInDefenseArea.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IAttackerTouchedBallInDefenseArea);

            /** AttackerTouchedBallInDefenseArea byTeam. */
            public byTeam: proto.Team;

            /** AttackerTouchedBallInDefenseArea byBot. */
            public byBot: number;

            /** AttackerTouchedBallInDefenseArea location. */
            public location?: (proto.IVector2|null);

            /** AttackerTouchedBallInDefenseArea distance. */
            public distance: number;

            /**
             * Creates a new AttackerTouchedBallInDefenseArea instance using the specified properties.
             * @param [properties] Properties to set
             * @returns AttackerTouchedBallInDefenseArea instance
             */
            public static create(properties?: proto.GameEvent.IAttackerTouchedBallInDefenseArea): proto.GameEvent.AttackerTouchedBallInDefenseArea;

            /**
             * Encodes the specified AttackerTouchedBallInDefenseArea message. Does not implicitly {@link proto.GameEvent.AttackerTouchedBallInDefenseArea.verify|verify} messages.
             * @param message AttackerTouchedBallInDefenseArea message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IAttackerTouchedBallInDefenseArea, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified AttackerTouchedBallInDefenseArea message, length delimited. Does not implicitly {@link proto.GameEvent.AttackerTouchedBallInDefenseArea.verify|verify} messages.
             * @param message AttackerTouchedBallInDefenseArea message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IAttackerTouchedBallInDefenseArea, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes an AttackerTouchedBallInDefenseArea message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns AttackerTouchedBallInDefenseArea
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.AttackerTouchedBallInDefenseArea;

            /**
             * Decodes an AttackerTouchedBallInDefenseArea message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns AttackerTouchedBallInDefenseArea
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.AttackerTouchedBallInDefenseArea;

            /**
             * Verifies an AttackerTouchedBallInDefenseArea message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates an AttackerTouchedBallInDefenseArea message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns AttackerTouchedBallInDefenseArea
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.AttackerTouchedBallInDefenseArea;

            /**
             * Creates a plain object from an AttackerTouchedBallInDefenseArea message. Also converts values to other types if specified.
             * @param message AttackerTouchedBallInDefenseArea
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.AttackerTouchedBallInDefenseArea, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this AttackerTouchedBallInDefenseArea to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for AttackerTouchedBallInDefenseArea
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a BotKickedBallTooFast. */
        interface IBotKickedBallTooFast {

            /** BotKickedBallTooFast byTeam */
            byTeam: proto.Team;

            /** BotKickedBallTooFast byBot */
            byBot?: (number|null);

            /** BotKickedBallTooFast location */
            location?: (proto.IVector2|null);

            /** BotKickedBallTooFast initialBallSpeed */
            initialBallSpeed?: (number|null);

            /** BotKickedBallTooFast chipped */
            chipped?: (boolean|null);
        }

        /** Represents a BotKickedBallTooFast. */
        class BotKickedBallTooFast implements IBotKickedBallTooFast {

            /**
             * Constructs a new BotKickedBallTooFast.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IBotKickedBallTooFast);

            /** BotKickedBallTooFast byTeam. */
            public byTeam: proto.Team;

            /** BotKickedBallTooFast byBot. */
            public byBot: number;

            /** BotKickedBallTooFast location. */
            public location?: (proto.IVector2|null);

            /** BotKickedBallTooFast initialBallSpeed. */
            public initialBallSpeed: number;

            /** BotKickedBallTooFast chipped. */
            public chipped: boolean;

            /**
             * Creates a new BotKickedBallTooFast instance using the specified properties.
             * @param [properties] Properties to set
             * @returns BotKickedBallTooFast instance
             */
            public static create(properties?: proto.GameEvent.IBotKickedBallTooFast): proto.GameEvent.BotKickedBallTooFast;

            /**
             * Encodes the specified BotKickedBallTooFast message. Does not implicitly {@link proto.GameEvent.BotKickedBallTooFast.verify|verify} messages.
             * @param message BotKickedBallTooFast message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IBotKickedBallTooFast, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified BotKickedBallTooFast message, length delimited. Does not implicitly {@link proto.GameEvent.BotKickedBallTooFast.verify|verify} messages.
             * @param message BotKickedBallTooFast message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IBotKickedBallTooFast, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a BotKickedBallTooFast message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns BotKickedBallTooFast
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.BotKickedBallTooFast;

            /**
             * Decodes a BotKickedBallTooFast message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns BotKickedBallTooFast
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.BotKickedBallTooFast;

            /**
             * Verifies a BotKickedBallTooFast message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a BotKickedBallTooFast message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns BotKickedBallTooFast
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.BotKickedBallTooFast;

            /**
             * Creates a plain object from a BotKickedBallTooFast message. Also converts values to other types if specified.
             * @param message BotKickedBallTooFast
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.BotKickedBallTooFast, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this BotKickedBallTooFast to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for BotKickedBallTooFast
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a BotDribbledBallTooFar. */
        interface IBotDribbledBallTooFar {

            /** BotDribbledBallTooFar byTeam */
            byTeam: proto.Team;

            /** BotDribbledBallTooFar byBot */
            byBot?: (number|null);

            /** BotDribbledBallTooFar start */
            start?: (proto.IVector2|null);

            /** BotDribbledBallTooFar end */
            end?: (proto.IVector2|null);
        }

        /** Represents a BotDribbledBallTooFar. */
        class BotDribbledBallTooFar implements IBotDribbledBallTooFar {

            /**
             * Constructs a new BotDribbledBallTooFar.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IBotDribbledBallTooFar);

            /** BotDribbledBallTooFar byTeam. */
            public byTeam: proto.Team;

            /** BotDribbledBallTooFar byBot. */
            public byBot: number;

            /** BotDribbledBallTooFar start. */
            public start?: (proto.IVector2|null);

            /** BotDribbledBallTooFar end. */
            public end?: (proto.IVector2|null);

            /**
             * Creates a new BotDribbledBallTooFar instance using the specified properties.
             * @param [properties] Properties to set
             * @returns BotDribbledBallTooFar instance
             */
            public static create(properties?: proto.GameEvent.IBotDribbledBallTooFar): proto.GameEvent.BotDribbledBallTooFar;

            /**
             * Encodes the specified BotDribbledBallTooFar message. Does not implicitly {@link proto.GameEvent.BotDribbledBallTooFar.verify|verify} messages.
             * @param message BotDribbledBallTooFar message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IBotDribbledBallTooFar, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified BotDribbledBallTooFar message, length delimited. Does not implicitly {@link proto.GameEvent.BotDribbledBallTooFar.verify|verify} messages.
             * @param message BotDribbledBallTooFar message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IBotDribbledBallTooFar, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a BotDribbledBallTooFar message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns BotDribbledBallTooFar
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.BotDribbledBallTooFar;

            /**
             * Decodes a BotDribbledBallTooFar message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns BotDribbledBallTooFar
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.BotDribbledBallTooFar;

            /**
             * Verifies a BotDribbledBallTooFar message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a BotDribbledBallTooFar message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns BotDribbledBallTooFar
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.BotDribbledBallTooFar;

            /**
             * Creates a plain object from a BotDribbledBallTooFar message. Also converts values to other types if specified.
             * @param message BotDribbledBallTooFar
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.BotDribbledBallTooFar, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this BotDribbledBallTooFar to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for BotDribbledBallTooFar
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of an AttackerTouchedOpponentInDefenseArea. */
        interface IAttackerTouchedOpponentInDefenseArea {

            /** AttackerTouchedOpponentInDefenseArea byTeam */
            byTeam: proto.Team;

            /** AttackerTouchedOpponentInDefenseArea byBot */
            byBot?: (number|null);

            /** AttackerTouchedOpponentInDefenseArea victim */
            victim?: (number|null);

            /** AttackerTouchedOpponentInDefenseArea location */
            location?: (proto.IVector2|null);
        }

        /** Represents an AttackerTouchedOpponentInDefenseArea. */
        class AttackerTouchedOpponentInDefenseArea implements IAttackerTouchedOpponentInDefenseArea {

            /**
             * Constructs a new AttackerTouchedOpponentInDefenseArea.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IAttackerTouchedOpponentInDefenseArea);

            /** AttackerTouchedOpponentInDefenseArea byTeam. */
            public byTeam: proto.Team;

            /** AttackerTouchedOpponentInDefenseArea byBot. */
            public byBot: number;

            /** AttackerTouchedOpponentInDefenseArea victim. */
            public victim: number;

            /** AttackerTouchedOpponentInDefenseArea location. */
            public location?: (proto.IVector2|null);

            /**
             * Creates a new AttackerTouchedOpponentInDefenseArea instance using the specified properties.
             * @param [properties] Properties to set
             * @returns AttackerTouchedOpponentInDefenseArea instance
             */
            public static create(properties?: proto.GameEvent.IAttackerTouchedOpponentInDefenseArea): proto.GameEvent.AttackerTouchedOpponentInDefenseArea;

            /**
             * Encodes the specified AttackerTouchedOpponentInDefenseArea message. Does not implicitly {@link proto.GameEvent.AttackerTouchedOpponentInDefenseArea.verify|verify} messages.
             * @param message AttackerTouchedOpponentInDefenseArea message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IAttackerTouchedOpponentInDefenseArea, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified AttackerTouchedOpponentInDefenseArea message, length delimited. Does not implicitly {@link proto.GameEvent.AttackerTouchedOpponentInDefenseArea.verify|verify} messages.
             * @param message AttackerTouchedOpponentInDefenseArea message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IAttackerTouchedOpponentInDefenseArea, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes an AttackerTouchedOpponentInDefenseArea message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns AttackerTouchedOpponentInDefenseArea
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.AttackerTouchedOpponentInDefenseArea;

            /**
             * Decodes an AttackerTouchedOpponentInDefenseArea message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns AttackerTouchedOpponentInDefenseArea
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.AttackerTouchedOpponentInDefenseArea;

            /**
             * Verifies an AttackerTouchedOpponentInDefenseArea message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates an AttackerTouchedOpponentInDefenseArea message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns AttackerTouchedOpponentInDefenseArea
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.AttackerTouchedOpponentInDefenseArea;

            /**
             * Creates a plain object from an AttackerTouchedOpponentInDefenseArea message. Also converts values to other types if specified.
             * @param message AttackerTouchedOpponentInDefenseArea
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.AttackerTouchedOpponentInDefenseArea, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this AttackerTouchedOpponentInDefenseArea to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for AttackerTouchedOpponentInDefenseArea
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of an AttackerDoubleTouchedBall. */
        interface IAttackerDoubleTouchedBall {

            /** AttackerDoubleTouchedBall byTeam */
            byTeam: proto.Team;

            /** AttackerDoubleTouchedBall byBot */
            byBot?: (number|null);

            /** AttackerDoubleTouchedBall location */
            location?: (proto.IVector2|null);
        }

        /** Represents an AttackerDoubleTouchedBall. */
        class AttackerDoubleTouchedBall implements IAttackerDoubleTouchedBall {

            /**
             * Constructs a new AttackerDoubleTouchedBall.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IAttackerDoubleTouchedBall);

            /** AttackerDoubleTouchedBall byTeam. */
            public byTeam: proto.Team;

            /** AttackerDoubleTouchedBall byBot. */
            public byBot: number;

            /** AttackerDoubleTouchedBall location. */
            public location?: (proto.IVector2|null);

            /**
             * Creates a new AttackerDoubleTouchedBall instance using the specified properties.
             * @param [properties] Properties to set
             * @returns AttackerDoubleTouchedBall instance
             */
            public static create(properties?: proto.GameEvent.IAttackerDoubleTouchedBall): proto.GameEvent.AttackerDoubleTouchedBall;

            /**
             * Encodes the specified AttackerDoubleTouchedBall message. Does not implicitly {@link proto.GameEvent.AttackerDoubleTouchedBall.verify|verify} messages.
             * @param message AttackerDoubleTouchedBall message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IAttackerDoubleTouchedBall, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified AttackerDoubleTouchedBall message, length delimited. Does not implicitly {@link proto.GameEvent.AttackerDoubleTouchedBall.verify|verify} messages.
             * @param message AttackerDoubleTouchedBall message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IAttackerDoubleTouchedBall, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes an AttackerDoubleTouchedBall message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns AttackerDoubleTouchedBall
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.AttackerDoubleTouchedBall;

            /**
             * Decodes an AttackerDoubleTouchedBall message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns AttackerDoubleTouchedBall
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.AttackerDoubleTouchedBall;

            /**
             * Verifies an AttackerDoubleTouchedBall message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates an AttackerDoubleTouchedBall message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns AttackerDoubleTouchedBall
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.AttackerDoubleTouchedBall;

            /**
             * Creates a plain object from an AttackerDoubleTouchedBall message. Also converts values to other types if specified.
             * @param message AttackerDoubleTouchedBall
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.AttackerDoubleTouchedBall, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this AttackerDoubleTouchedBall to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for AttackerDoubleTouchedBall
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of an AttackerTooCloseToDefenseArea. */
        interface IAttackerTooCloseToDefenseArea {

            /** AttackerTooCloseToDefenseArea byTeam */
            byTeam: proto.Team;

            /** AttackerTooCloseToDefenseArea byBot */
            byBot?: (number|null);

            /** AttackerTooCloseToDefenseArea location */
            location?: (proto.IVector2|null);

            /** AttackerTooCloseToDefenseArea distance */
            distance?: (number|null);

            /** AttackerTooCloseToDefenseArea ballLocation */
            ballLocation?: (proto.IVector2|null);
        }

        /** Represents an AttackerTooCloseToDefenseArea. */
        class AttackerTooCloseToDefenseArea implements IAttackerTooCloseToDefenseArea {

            /**
             * Constructs a new AttackerTooCloseToDefenseArea.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IAttackerTooCloseToDefenseArea);

            /** AttackerTooCloseToDefenseArea byTeam. */
            public byTeam: proto.Team;

            /** AttackerTooCloseToDefenseArea byBot. */
            public byBot: number;

            /** AttackerTooCloseToDefenseArea location. */
            public location?: (proto.IVector2|null);

            /** AttackerTooCloseToDefenseArea distance. */
            public distance: number;

            /** AttackerTooCloseToDefenseArea ballLocation. */
            public ballLocation?: (proto.IVector2|null);

            /**
             * Creates a new AttackerTooCloseToDefenseArea instance using the specified properties.
             * @param [properties] Properties to set
             * @returns AttackerTooCloseToDefenseArea instance
             */
            public static create(properties?: proto.GameEvent.IAttackerTooCloseToDefenseArea): proto.GameEvent.AttackerTooCloseToDefenseArea;

            /**
             * Encodes the specified AttackerTooCloseToDefenseArea message. Does not implicitly {@link proto.GameEvent.AttackerTooCloseToDefenseArea.verify|verify} messages.
             * @param message AttackerTooCloseToDefenseArea message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IAttackerTooCloseToDefenseArea, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified AttackerTooCloseToDefenseArea message, length delimited. Does not implicitly {@link proto.GameEvent.AttackerTooCloseToDefenseArea.verify|verify} messages.
             * @param message AttackerTooCloseToDefenseArea message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IAttackerTooCloseToDefenseArea, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes an AttackerTooCloseToDefenseArea message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns AttackerTooCloseToDefenseArea
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.AttackerTooCloseToDefenseArea;

            /**
             * Decodes an AttackerTooCloseToDefenseArea message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns AttackerTooCloseToDefenseArea
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.AttackerTooCloseToDefenseArea;

            /**
             * Verifies an AttackerTooCloseToDefenseArea message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates an AttackerTooCloseToDefenseArea message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns AttackerTooCloseToDefenseArea
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.AttackerTooCloseToDefenseArea;

            /**
             * Creates a plain object from an AttackerTooCloseToDefenseArea message. Also converts values to other types if specified.
             * @param message AttackerTooCloseToDefenseArea
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.AttackerTooCloseToDefenseArea, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this AttackerTooCloseToDefenseArea to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for AttackerTooCloseToDefenseArea
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a BotHeldBallDeliberately. */
        interface IBotHeldBallDeliberately {

            /** BotHeldBallDeliberately byTeam */
            byTeam: proto.Team;

            /** BotHeldBallDeliberately byBot */
            byBot?: (number|null);

            /** BotHeldBallDeliberately location */
            location?: (proto.IVector2|null);

            /** BotHeldBallDeliberately duration */
            duration?: (number|null);
        }

        /** Represents a BotHeldBallDeliberately. */
        class BotHeldBallDeliberately implements IBotHeldBallDeliberately {

            /**
             * Constructs a new BotHeldBallDeliberately.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IBotHeldBallDeliberately);

            /** BotHeldBallDeliberately byTeam. */
            public byTeam: proto.Team;

            /** BotHeldBallDeliberately byBot. */
            public byBot: number;

            /** BotHeldBallDeliberately location. */
            public location?: (proto.IVector2|null);

            /** BotHeldBallDeliberately duration. */
            public duration: number;

            /**
             * Creates a new BotHeldBallDeliberately instance using the specified properties.
             * @param [properties] Properties to set
             * @returns BotHeldBallDeliberately instance
             */
            public static create(properties?: proto.GameEvent.IBotHeldBallDeliberately): proto.GameEvent.BotHeldBallDeliberately;

            /**
             * Encodes the specified BotHeldBallDeliberately message. Does not implicitly {@link proto.GameEvent.BotHeldBallDeliberately.verify|verify} messages.
             * @param message BotHeldBallDeliberately message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IBotHeldBallDeliberately, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified BotHeldBallDeliberately message, length delimited. Does not implicitly {@link proto.GameEvent.BotHeldBallDeliberately.verify|verify} messages.
             * @param message BotHeldBallDeliberately message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IBotHeldBallDeliberately, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a BotHeldBallDeliberately message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns BotHeldBallDeliberately
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.BotHeldBallDeliberately;

            /**
             * Decodes a BotHeldBallDeliberately message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns BotHeldBallDeliberately
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.BotHeldBallDeliberately;

            /**
             * Verifies a BotHeldBallDeliberately message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a BotHeldBallDeliberately message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns BotHeldBallDeliberately
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.BotHeldBallDeliberately;

            /**
             * Creates a plain object from a BotHeldBallDeliberately message. Also converts values to other types if specified.
             * @param message BotHeldBallDeliberately
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.BotHeldBallDeliberately, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this BotHeldBallDeliberately to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for BotHeldBallDeliberately
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a BotInterferedPlacement. */
        interface IBotInterferedPlacement {

            /** BotInterferedPlacement byTeam */
            byTeam: proto.Team;

            /** BotInterferedPlacement byBot */
            byBot?: (number|null);

            /** BotInterferedPlacement location */
            location?: (proto.IVector2|null);
        }

        /** Represents a BotInterferedPlacement. */
        class BotInterferedPlacement implements IBotInterferedPlacement {

            /**
             * Constructs a new BotInterferedPlacement.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IBotInterferedPlacement);

            /** BotInterferedPlacement byTeam. */
            public byTeam: proto.Team;

            /** BotInterferedPlacement byBot. */
            public byBot: number;

            /** BotInterferedPlacement location. */
            public location?: (proto.IVector2|null);

            /**
             * Creates a new BotInterferedPlacement instance using the specified properties.
             * @param [properties] Properties to set
             * @returns BotInterferedPlacement instance
             */
            public static create(properties?: proto.GameEvent.IBotInterferedPlacement): proto.GameEvent.BotInterferedPlacement;

            /**
             * Encodes the specified BotInterferedPlacement message. Does not implicitly {@link proto.GameEvent.BotInterferedPlacement.verify|verify} messages.
             * @param message BotInterferedPlacement message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IBotInterferedPlacement, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified BotInterferedPlacement message, length delimited. Does not implicitly {@link proto.GameEvent.BotInterferedPlacement.verify|verify} messages.
             * @param message BotInterferedPlacement message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IBotInterferedPlacement, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a BotInterferedPlacement message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns BotInterferedPlacement
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.BotInterferedPlacement;

            /**
             * Decodes a BotInterferedPlacement message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns BotInterferedPlacement
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.BotInterferedPlacement;

            /**
             * Verifies a BotInterferedPlacement message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a BotInterferedPlacement message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns BotInterferedPlacement
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.BotInterferedPlacement;

            /**
             * Creates a plain object from a BotInterferedPlacement message. Also converts values to other types if specified.
             * @param message BotInterferedPlacement
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.BotInterferedPlacement, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this BotInterferedPlacement to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for BotInterferedPlacement
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a MultipleCards. */
        interface IMultipleCards {

            /** MultipleCards byTeam */
            byTeam: proto.Team;
        }

        /** Represents a MultipleCards. */
        class MultipleCards implements IMultipleCards {

            /**
             * Constructs a new MultipleCards.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IMultipleCards);

            /** MultipleCards byTeam. */
            public byTeam: proto.Team;

            /**
             * Creates a new MultipleCards instance using the specified properties.
             * @param [properties] Properties to set
             * @returns MultipleCards instance
             */
            public static create(properties?: proto.GameEvent.IMultipleCards): proto.GameEvent.MultipleCards;

            /**
             * Encodes the specified MultipleCards message. Does not implicitly {@link proto.GameEvent.MultipleCards.verify|verify} messages.
             * @param message MultipleCards message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IMultipleCards, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified MultipleCards message, length delimited. Does not implicitly {@link proto.GameEvent.MultipleCards.verify|verify} messages.
             * @param message MultipleCards message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IMultipleCards, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a MultipleCards message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns MultipleCards
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.MultipleCards;

            /**
             * Decodes a MultipleCards message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns MultipleCards
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.MultipleCards;

            /**
             * Verifies a MultipleCards message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a MultipleCards message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns MultipleCards
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.MultipleCards;

            /**
             * Creates a plain object from a MultipleCards message. Also converts values to other types if specified.
             * @param message MultipleCards
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.MultipleCards, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this MultipleCards to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for MultipleCards
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a MultipleFouls. */
        interface IMultipleFouls {

            /** MultipleFouls byTeam */
            byTeam: proto.Team;

            /** MultipleFouls causedGameEvents */
            causedGameEvents?: (proto.IGameEvent[]|null);
        }

        /** Represents a MultipleFouls. */
        class MultipleFouls implements IMultipleFouls {

            /**
             * Constructs a new MultipleFouls.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IMultipleFouls);

            /** MultipleFouls byTeam. */
            public byTeam: proto.Team;

            /** MultipleFouls causedGameEvents. */
            public causedGameEvents: proto.IGameEvent[];

            /**
             * Creates a new MultipleFouls instance using the specified properties.
             * @param [properties] Properties to set
             * @returns MultipleFouls instance
             */
            public static create(properties?: proto.GameEvent.IMultipleFouls): proto.GameEvent.MultipleFouls;

            /**
             * Encodes the specified MultipleFouls message. Does not implicitly {@link proto.GameEvent.MultipleFouls.verify|verify} messages.
             * @param message MultipleFouls message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IMultipleFouls, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified MultipleFouls message, length delimited. Does not implicitly {@link proto.GameEvent.MultipleFouls.verify|verify} messages.
             * @param message MultipleFouls message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IMultipleFouls, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a MultipleFouls message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns MultipleFouls
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.MultipleFouls;

            /**
             * Decodes a MultipleFouls message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns MultipleFouls
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.MultipleFouls;

            /**
             * Verifies a MultipleFouls message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a MultipleFouls message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns MultipleFouls
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.MultipleFouls;

            /**
             * Creates a plain object from a MultipleFouls message. Also converts values to other types if specified.
             * @param message MultipleFouls
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.MultipleFouls, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this MultipleFouls to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for MultipleFouls
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a MultiplePlacementFailures. */
        interface IMultiplePlacementFailures {

            /** MultiplePlacementFailures byTeam */
            byTeam: proto.Team;
        }

        /** Represents a MultiplePlacementFailures. */
        class MultiplePlacementFailures implements IMultiplePlacementFailures {

            /**
             * Constructs a new MultiplePlacementFailures.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IMultiplePlacementFailures);

            /** MultiplePlacementFailures byTeam. */
            public byTeam: proto.Team;

            /**
             * Creates a new MultiplePlacementFailures instance using the specified properties.
             * @param [properties] Properties to set
             * @returns MultiplePlacementFailures instance
             */
            public static create(properties?: proto.GameEvent.IMultiplePlacementFailures): proto.GameEvent.MultiplePlacementFailures;

            /**
             * Encodes the specified MultiplePlacementFailures message. Does not implicitly {@link proto.GameEvent.MultiplePlacementFailures.verify|verify} messages.
             * @param message MultiplePlacementFailures message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IMultiplePlacementFailures, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified MultiplePlacementFailures message, length delimited. Does not implicitly {@link proto.GameEvent.MultiplePlacementFailures.verify|verify} messages.
             * @param message MultiplePlacementFailures message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IMultiplePlacementFailures, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a MultiplePlacementFailures message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns MultiplePlacementFailures
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.MultiplePlacementFailures;

            /**
             * Decodes a MultiplePlacementFailures message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns MultiplePlacementFailures
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.MultiplePlacementFailures;

            /**
             * Verifies a MultiplePlacementFailures message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a MultiplePlacementFailures message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns MultiplePlacementFailures
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.MultiplePlacementFailures;

            /**
             * Creates a plain object from a MultiplePlacementFailures message. Also converts values to other types if specified.
             * @param message MultiplePlacementFailures
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.MultiplePlacementFailures, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this MultiplePlacementFailures to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for MultiplePlacementFailures
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a KickTimeout. */
        interface IKickTimeout {

            /** KickTimeout byTeam */
            byTeam: proto.Team;

            /** KickTimeout location */
            location?: (proto.IVector2|null);

            /** KickTimeout time */
            time?: (number|null);
        }

        /** Represents a KickTimeout. */
        class KickTimeout implements IKickTimeout {

            /**
             * Constructs a new KickTimeout.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IKickTimeout);

            /** KickTimeout byTeam. */
            public byTeam: proto.Team;

            /** KickTimeout location. */
            public location?: (proto.IVector2|null);

            /** KickTimeout time. */
            public time: number;

            /**
             * Creates a new KickTimeout instance using the specified properties.
             * @param [properties] Properties to set
             * @returns KickTimeout instance
             */
            public static create(properties?: proto.GameEvent.IKickTimeout): proto.GameEvent.KickTimeout;

            /**
             * Encodes the specified KickTimeout message. Does not implicitly {@link proto.GameEvent.KickTimeout.verify|verify} messages.
             * @param message KickTimeout message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IKickTimeout, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified KickTimeout message, length delimited. Does not implicitly {@link proto.GameEvent.KickTimeout.verify|verify} messages.
             * @param message KickTimeout message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IKickTimeout, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a KickTimeout message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns KickTimeout
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.KickTimeout;

            /**
             * Decodes a KickTimeout message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns KickTimeout
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.KickTimeout;

            /**
             * Verifies a KickTimeout message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a KickTimeout message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns KickTimeout
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.KickTimeout;

            /**
             * Creates a plain object from a KickTimeout message. Also converts values to other types if specified.
             * @param message KickTimeout
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.KickTimeout, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this KickTimeout to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for KickTimeout
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a NoProgressInGame. */
        interface INoProgressInGame {

            /** NoProgressInGame location */
            location?: (proto.IVector2|null);

            /** NoProgressInGame time */
            time?: (number|null);
        }

        /** Represents a NoProgressInGame. */
        class NoProgressInGame implements INoProgressInGame {

            /**
             * Constructs a new NoProgressInGame.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.INoProgressInGame);

            /** NoProgressInGame location. */
            public location?: (proto.IVector2|null);

            /** NoProgressInGame time. */
            public time: number;

            /**
             * Creates a new NoProgressInGame instance using the specified properties.
             * @param [properties] Properties to set
             * @returns NoProgressInGame instance
             */
            public static create(properties?: proto.GameEvent.INoProgressInGame): proto.GameEvent.NoProgressInGame;

            /**
             * Encodes the specified NoProgressInGame message. Does not implicitly {@link proto.GameEvent.NoProgressInGame.verify|verify} messages.
             * @param message NoProgressInGame message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.INoProgressInGame, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified NoProgressInGame message, length delimited. Does not implicitly {@link proto.GameEvent.NoProgressInGame.verify|verify} messages.
             * @param message NoProgressInGame message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.INoProgressInGame, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a NoProgressInGame message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns NoProgressInGame
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.NoProgressInGame;

            /**
             * Decodes a NoProgressInGame message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns NoProgressInGame
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.NoProgressInGame;

            /**
             * Verifies a NoProgressInGame message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a NoProgressInGame message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns NoProgressInGame
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.NoProgressInGame;

            /**
             * Creates a plain object from a NoProgressInGame message. Also converts values to other types if specified.
             * @param message NoProgressInGame
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.NoProgressInGame, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this NoProgressInGame to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for NoProgressInGame
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a PlacementFailed. */
        interface IPlacementFailed {

            /** PlacementFailed byTeam */
            byTeam: proto.Team;

            /** PlacementFailed remainingDistance */
            remainingDistance?: (number|null);
        }

        /** Represents a PlacementFailed. */
        class PlacementFailed implements IPlacementFailed {

            /**
             * Constructs a new PlacementFailed.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IPlacementFailed);

            /** PlacementFailed byTeam. */
            public byTeam: proto.Team;

            /** PlacementFailed remainingDistance. */
            public remainingDistance: number;

            /**
             * Creates a new PlacementFailed instance using the specified properties.
             * @param [properties] Properties to set
             * @returns PlacementFailed instance
             */
            public static create(properties?: proto.GameEvent.IPlacementFailed): proto.GameEvent.PlacementFailed;

            /**
             * Encodes the specified PlacementFailed message. Does not implicitly {@link proto.GameEvent.PlacementFailed.verify|verify} messages.
             * @param message PlacementFailed message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IPlacementFailed, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified PlacementFailed message, length delimited. Does not implicitly {@link proto.GameEvent.PlacementFailed.verify|verify} messages.
             * @param message PlacementFailed message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IPlacementFailed, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a PlacementFailed message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns PlacementFailed
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.PlacementFailed;

            /**
             * Decodes a PlacementFailed message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns PlacementFailed
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.PlacementFailed;

            /**
             * Verifies a PlacementFailed message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a PlacementFailed message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns PlacementFailed
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.PlacementFailed;

            /**
             * Creates a plain object from a PlacementFailed message. Also converts values to other types if specified.
             * @param message PlacementFailed
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.PlacementFailed, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this PlacementFailed to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for PlacementFailed
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of an UnsportingBehaviorMinor. */
        interface IUnsportingBehaviorMinor {

            /** UnsportingBehaviorMinor byTeam */
            byTeam: proto.Team;

            /** UnsportingBehaviorMinor reason */
            reason: string;
        }

        /** Represents an UnsportingBehaviorMinor. */
        class UnsportingBehaviorMinor implements IUnsportingBehaviorMinor {

            /**
             * Constructs a new UnsportingBehaviorMinor.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IUnsportingBehaviorMinor);

            /** UnsportingBehaviorMinor byTeam. */
            public byTeam: proto.Team;

            /** UnsportingBehaviorMinor reason. */
            public reason: string;

            /**
             * Creates a new UnsportingBehaviorMinor instance using the specified properties.
             * @param [properties] Properties to set
             * @returns UnsportingBehaviorMinor instance
             */
            public static create(properties?: proto.GameEvent.IUnsportingBehaviorMinor): proto.GameEvent.UnsportingBehaviorMinor;

            /**
             * Encodes the specified UnsportingBehaviorMinor message. Does not implicitly {@link proto.GameEvent.UnsportingBehaviorMinor.verify|verify} messages.
             * @param message UnsportingBehaviorMinor message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IUnsportingBehaviorMinor, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified UnsportingBehaviorMinor message, length delimited. Does not implicitly {@link proto.GameEvent.UnsportingBehaviorMinor.verify|verify} messages.
             * @param message UnsportingBehaviorMinor message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IUnsportingBehaviorMinor, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes an UnsportingBehaviorMinor message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns UnsportingBehaviorMinor
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.UnsportingBehaviorMinor;

            /**
             * Decodes an UnsportingBehaviorMinor message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns UnsportingBehaviorMinor
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.UnsportingBehaviorMinor;

            /**
             * Verifies an UnsportingBehaviorMinor message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates an UnsportingBehaviorMinor message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns UnsportingBehaviorMinor
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.UnsportingBehaviorMinor;

            /**
             * Creates a plain object from an UnsportingBehaviorMinor message. Also converts values to other types if specified.
             * @param message UnsportingBehaviorMinor
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.UnsportingBehaviorMinor, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this UnsportingBehaviorMinor to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for UnsportingBehaviorMinor
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of an UnsportingBehaviorMajor. */
        interface IUnsportingBehaviorMajor {

            /** UnsportingBehaviorMajor byTeam */
            byTeam: proto.Team;

            /** UnsportingBehaviorMajor reason */
            reason: string;
        }

        /** Represents an UnsportingBehaviorMajor. */
        class UnsportingBehaviorMajor implements IUnsportingBehaviorMajor {

            /**
             * Constructs a new UnsportingBehaviorMajor.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IUnsportingBehaviorMajor);

            /** UnsportingBehaviorMajor byTeam. */
            public byTeam: proto.Team;

            /** UnsportingBehaviorMajor reason. */
            public reason: string;

            /**
             * Creates a new UnsportingBehaviorMajor instance using the specified properties.
             * @param [properties] Properties to set
             * @returns UnsportingBehaviorMajor instance
             */
            public static create(properties?: proto.GameEvent.IUnsportingBehaviorMajor): proto.GameEvent.UnsportingBehaviorMajor;

            /**
             * Encodes the specified UnsportingBehaviorMajor message. Does not implicitly {@link proto.GameEvent.UnsportingBehaviorMajor.verify|verify} messages.
             * @param message UnsportingBehaviorMajor message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IUnsportingBehaviorMajor, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified UnsportingBehaviorMajor message, length delimited. Does not implicitly {@link proto.GameEvent.UnsportingBehaviorMajor.verify|verify} messages.
             * @param message UnsportingBehaviorMajor message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IUnsportingBehaviorMajor, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes an UnsportingBehaviorMajor message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns UnsportingBehaviorMajor
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.UnsportingBehaviorMajor;

            /**
             * Decodes an UnsportingBehaviorMajor message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns UnsportingBehaviorMajor
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.UnsportingBehaviorMajor;

            /**
             * Verifies an UnsportingBehaviorMajor message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates an UnsportingBehaviorMajor message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns UnsportingBehaviorMajor
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.UnsportingBehaviorMajor;

            /**
             * Creates a plain object from an UnsportingBehaviorMajor message. Also converts values to other types if specified.
             * @param message UnsportingBehaviorMajor
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.UnsportingBehaviorMajor, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this UnsportingBehaviorMajor to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for UnsportingBehaviorMajor
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a KeeperHeldBall. */
        interface IKeeperHeldBall {

            /** KeeperHeldBall byTeam */
            byTeam: proto.Team;

            /** KeeperHeldBall location */
            location?: (proto.IVector2|null);

            /** KeeperHeldBall duration */
            duration?: (number|null);
        }

        /** Represents a KeeperHeldBall. */
        class KeeperHeldBall implements IKeeperHeldBall {

            /**
             * Constructs a new KeeperHeldBall.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IKeeperHeldBall);

            /** KeeperHeldBall byTeam. */
            public byTeam: proto.Team;

            /** KeeperHeldBall location. */
            public location?: (proto.IVector2|null);

            /** KeeperHeldBall duration. */
            public duration: number;

            /**
             * Creates a new KeeperHeldBall instance using the specified properties.
             * @param [properties] Properties to set
             * @returns KeeperHeldBall instance
             */
            public static create(properties?: proto.GameEvent.IKeeperHeldBall): proto.GameEvent.KeeperHeldBall;

            /**
             * Encodes the specified KeeperHeldBall message. Does not implicitly {@link proto.GameEvent.KeeperHeldBall.verify|verify} messages.
             * @param message KeeperHeldBall message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IKeeperHeldBall, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified KeeperHeldBall message, length delimited. Does not implicitly {@link proto.GameEvent.KeeperHeldBall.verify|verify} messages.
             * @param message KeeperHeldBall message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IKeeperHeldBall, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a KeeperHeldBall message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns KeeperHeldBall
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.KeeperHeldBall;

            /**
             * Decodes a KeeperHeldBall message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns KeeperHeldBall
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.KeeperHeldBall;

            /**
             * Verifies a KeeperHeldBall message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a KeeperHeldBall message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns KeeperHeldBall
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.KeeperHeldBall;

            /**
             * Creates a plain object from a KeeperHeldBall message. Also converts values to other types if specified.
             * @param message KeeperHeldBall
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.KeeperHeldBall, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this KeeperHeldBall to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for KeeperHeldBall
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a PlacementSucceeded. */
        interface IPlacementSucceeded {

            /** PlacementSucceeded byTeam */
            byTeam: proto.Team;

            /** PlacementSucceeded timeTaken */
            timeTaken?: (number|null);

            /** PlacementSucceeded precision */
            precision?: (number|null);

            /** PlacementSucceeded distance */
            distance?: (number|null);
        }

        /** Represents a PlacementSucceeded. */
        class PlacementSucceeded implements IPlacementSucceeded {

            /**
             * Constructs a new PlacementSucceeded.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IPlacementSucceeded);

            /** PlacementSucceeded byTeam. */
            public byTeam: proto.Team;

            /** PlacementSucceeded timeTaken. */
            public timeTaken: number;

            /** PlacementSucceeded precision. */
            public precision: number;

            /** PlacementSucceeded distance. */
            public distance: number;

            /**
             * Creates a new PlacementSucceeded instance using the specified properties.
             * @param [properties] Properties to set
             * @returns PlacementSucceeded instance
             */
            public static create(properties?: proto.GameEvent.IPlacementSucceeded): proto.GameEvent.PlacementSucceeded;

            /**
             * Encodes the specified PlacementSucceeded message. Does not implicitly {@link proto.GameEvent.PlacementSucceeded.verify|verify} messages.
             * @param message PlacementSucceeded message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IPlacementSucceeded, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified PlacementSucceeded message, length delimited. Does not implicitly {@link proto.GameEvent.PlacementSucceeded.verify|verify} messages.
             * @param message PlacementSucceeded message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IPlacementSucceeded, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a PlacementSucceeded message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns PlacementSucceeded
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.PlacementSucceeded;

            /**
             * Decodes a PlacementSucceeded message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns PlacementSucceeded
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.PlacementSucceeded;

            /**
             * Verifies a PlacementSucceeded message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a PlacementSucceeded message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns PlacementSucceeded
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.PlacementSucceeded;

            /**
             * Creates a plain object from a PlacementSucceeded message. Also converts values to other types if specified.
             * @param message PlacementSucceeded
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.PlacementSucceeded, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this PlacementSucceeded to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for PlacementSucceeded
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a Prepared. */
        interface IPrepared {

            /** Prepared timeTaken */
            timeTaken?: (number|null);
        }

        /** Represents a Prepared. */
        class Prepared implements IPrepared {

            /**
             * Constructs a new Prepared.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IPrepared);

            /** Prepared timeTaken. */
            public timeTaken: number;

            /**
             * Creates a new Prepared instance using the specified properties.
             * @param [properties] Properties to set
             * @returns Prepared instance
             */
            public static create(properties?: proto.GameEvent.IPrepared): proto.GameEvent.Prepared;

            /**
             * Encodes the specified Prepared message. Does not implicitly {@link proto.GameEvent.Prepared.verify|verify} messages.
             * @param message Prepared message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IPrepared, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified Prepared message, length delimited. Does not implicitly {@link proto.GameEvent.Prepared.verify|verify} messages.
             * @param message Prepared message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IPrepared, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a Prepared message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns Prepared
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.Prepared;

            /**
             * Decodes a Prepared message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns Prepared
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.Prepared;

            /**
             * Verifies a Prepared message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a Prepared message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns Prepared
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.Prepared;

            /**
             * Creates a plain object from a Prepared message. Also converts values to other types if specified.
             * @param message Prepared
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.Prepared, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this Prepared to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for Prepared
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a BotSubstitution. */
        interface IBotSubstitution {

            /** BotSubstitution byTeam */
            byTeam: proto.Team;
        }

        /** Represents a BotSubstitution. */
        class BotSubstitution implements IBotSubstitution {

            /**
             * Constructs a new BotSubstitution.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IBotSubstitution);

            /** BotSubstitution byTeam. */
            public byTeam: proto.Team;

            /**
             * Creates a new BotSubstitution instance using the specified properties.
             * @param [properties] Properties to set
             * @returns BotSubstitution instance
             */
            public static create(properties?: proto.GameEvent.IBotSubstitution): proto.GameEvent.BotSubstitution;

            /**
             * Encodes the specified BotSubstitution message. Does not implicitly {@link proto.GameEvent.BotSubstitution.verify|verify} messages.
             * @param message BotSubstitution message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IBotSubstitution, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified BotSubstitution message, length delimited. Does not implicitly {@link proto.GameEvent.BotSubstitution.verify|verify} messages.
             * @param message BotSubstitution message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IBotSubstitution, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a BotSubstitution message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns BotSubstitution
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.BotSubstitution;

            /**
             * Decodes a BotSubstitution message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns BotSubstitution
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.BotSubstitution;

            /**
             * Verifies a BotSubstitution message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a BotSubstitution message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns BotSubstitution
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.BotSubstitution;

            /**
             * Creates a plain object from a BotSubstitution message. Also converts values to other types if specified.
             * @param message BotSubstitution
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.BotSubstitution, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this BotSubstitution to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for BotSubstitution
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a ChallengeFlag. */
        interface IChallengeFlag {

            /** ChallengeFlag byTeam */
            byTeam: proto.Team;
        }

        /** Represents a ChallengeFlag. */
        class ChallengeFlag implements IChallengeFlag {

            /**
             * Constructs a new ChallengeFlag.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IChallengeFlag);

            /** ChallengeFlag byTeam. */
            public byTeam: proto.Team;

            /**
             * Creates a new ChallengeFlag instance using the specified properties.
             * @param [properties] Properties to set
             * @returns ChallengeFlag instance
             */
            public static create(properties?: proto.GameEvent.IChallengeFlag): proto.GameEvent.ChallengeFlag;

            /**
             * Encodes the specified ChallengeFlag message. Does not implicitly {@link proto.GameEvent.ChallengeFlag.verify|verify} messages.
             * @param message ChallengeFlag message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IChallengeFlag, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified ChallengeFlag message, length delimited. Does not implicitly {@link proto.GameEvent.ChallengeFlag.verify|verify} messages.
             * @param message ChallengeFlag message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IChallengeFlag, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a ChallengeFlag message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns ChallengeFlag
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.ChallengeFlag;

            /**
             * Decodes a ChallengeFlag message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns ChallengeFlag
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.ChallengeFlag;

            /**
             * Verifies a ChallengeFlag message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a ChallengeFlag message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns ChallengeFlag
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.ChallengeFlag;

            /**
             * Creates a plain object from a ChallengeFlag message. Also converts values to other types if specified.
             * @param message ChallengeFlag
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.ChallengeFlag, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this ChallengeFlag to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for ChallengeFlag
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a ChallengeFlagHandled. */
        interface IChallengeFlagHandled {

            /** ChallengeFlagHandled byTeam */
            byTeam: proto.Team;

            /** ChallengeFlagHandled accepted */
            accepted: boolean;
        }

        /** Represents a ChallengeFlagHandled. */
        class ChallengeFlagHandled implements IChallengeFlagHandled {

            /**
             * Constructs a new ChallengeFlagHandled.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IChallengeFlagHandled);

            /** ChallengeFlagHandled byTeam. */
            public byTeam: proto.Team;

            /** ChallengeFlagHandled accepted. */
            public accepted: boolean;

            /**
             * Creates a new ChallengeFlagHandled instance using the specified properties.
             * @param [properties] Properties to set
             * @returns ChallengeFlagHandled instance
             */
            public static create(properties?: proto.GameEvent.IChallengeFlagHandled): proto.GameEvent.ChallengeFlagHandled;

            /**
             * Encodes the specified ChallengeFlagHandled message. Does not implicitly {@link proto.GameEvent.ChallengeFlagHandled.verify|verify} messages.
             * @param message ChallengeFlagHandled message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IChallengeFlagHandled, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified ChallengeFlagHandled message, length delimited. Does not implicitly {@link proto.GameEvent.ChallengeFlagHandled.verify|verify} messages.
             * @param message ChallengeFlagHandled message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IChallengeFlagHandled, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a ChallengeFlagHandled message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns ChallengeFlagHandled
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.ChallengeFlagHandled;

            /**
             * Decodes a ChallengeFlagHandled message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns ChallengeFlagHandled
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.ChallengeFlagHandled;

            /**
             * Verifies a ChallengeFlagHandled message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a ChallengeFlagHandled message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns ChallengeFlagHandled
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.ChallengeFlagHandled;

            /**
             * Creates a plain object from a ChallengeFlagHandled message. Also converts values to other types if specified.
             * @param message ChallengeFlagHandled
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.ChallengeFlagHandled, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this ChallengeFlagHandled to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for ChallengeFlagHandled
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of an EmergencyStop. */
        interface IEmergencyStop {

            /** EmergencyStop byTeam */
            byTeam: proto.Team;
        }

        /** Represents an EmergencyStop. */
        class EmergencyStop implements IEmergencyStop {

            /**
             * Constructs a new EmergencyStop.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IEmergencyStop);

            /** EmergencyStop byTeam. */
            public byTeam: proto.Team;

            /**
             * Creates a new EmergencyStop instance using the specified properties.
             * @param [properties] Properties to set
             * @returns EmergencyStop instance
             */
            public static create(properties?: proto.GameEvent.IEmergencyStop): proto.GameEvent.EmergencyStop;

            /**
             * Encodes the specified EmergencyStop message. Does not implicitly {@link proto.GameEvent.EmergencyStop.verify|verify} messages.
             * @param message EmergencyStop message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IEmergencyStop, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified EmergencyStop message, length delimited. Does not implicitly {@link proto.GameEvent.EmergencyStop.verify|verify} messages.
             * @param message EmergencyStop message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IEmergencyStop, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes an EmergencyStop message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns EmergencyStop
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.EmergencyStop;

            /**
             * Decodes an EmergencyStop message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns EmergencyStop
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.EmergencyStop;

            /**
             * Verifies an EmergencyStop message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates an EmergencyStop message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns EmergencyStop
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.EmergencyStop;

            /**
             * Creates a plain object from an EmergencyStop message. Also converts values to other types if specified.
             * @param message EmergencyStop
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.EmergencyStop, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this EmergencyStop to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for EmergencyStop
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a TooManyRobots. */
        interface ITooManyRobots {

            /** TooManyRobots byTeam */
            byTeam: proto.Team;

            /** TooManyRobots numRobotsAllowed */
            numRobotsAllowed?: (number|null);

            /** TooManyRobots numRobotsOnField */
            numRobotsOnField?: (number|null);

            /** TooManyRobots ballLocation */
            ballLocation?: (proto.IVector2|null);
        }

        /** Represents a TooManyRobots. */
        class TooManyRobots implements ITooManyRobots {

            /**
             * Constructs a new TooManyRobots.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.ITooManyRobots);

            /** TooManyRobots byTeam. */
            public byTeam: proto.Team;

            /** TooManyRobots numRobotsAllowed. */
            public numRobotsAllowed: number;

            /** TooManyRobots numRobotsOnField. */
            public numRobotsOnField: number;

            /** TooManyRobots ballLocation. */
            public ballLocation?: (proto.IVector2|null);

            /**
             * Creates a new TooManyRobots instance using the specified properties.
             * @param [properties] Properties to set
             * @returns TooManyRobots instance
             */
            public static create(properties?: proto.GameEvent.ITooManyRobots): proto.GameEvent.TooManyRobots;

            /**
             * Encodes the specified TooManyRobots message. Does not implicitly {@link proto.GameEvent.TooManyRobots.verify|verify} messages.
             * @param message TooManyRobots message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.ITooManyRobots, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified TooManyRobots message, length delimited. Does not implicitly {@link proto.GameEvent.TooManyRobots.verify|verify} messages.
             * @param message TooManyRobots message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.ITooManyRobots, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a TooManyRobots message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns TooManyRobots
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.TooManyRobots;

            /**
             * Decodes a TooManyRobots message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns TooManyRobots
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.TooManyRobots;

            /**
             * Verifies a TooManyRobots message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a TooManyRobots message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns TooManyRobots
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.TooManyRobots;

            /**
             * Creates a plain object from a TooManyRobots message. Also converts values to other types if specified.
             * @param message TooManyRobots
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.TooManyRobots, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this TooManyRobots to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for TooManyRobots
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a BoundaryCrossing. */
        interface IBoundaryCrossing {

            /** BoundaryCrossing byTeam */
            byTeam: proto.Team;

            /** BoundaryCrossing location */
            location?: (proto.IVector2|null);
        }

        /** Represents a BoundaryCrossing. */
        class BoundaryCrossing implements IBoundaryCrossing {

            /**
             * Constructs a new BoundaryCrossing.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IBoundaryCrossing);

            /** BoundaryCrossing byTeam. */
            public byTeam: proto.Team;

            /** BoundaryCrossing location. */
            public location?: (proto.IVector2|null);

            /**
             * Creates a new BoundaryCrossing instance using the specified properties.
             * @param [properties] Properties to set
             * @returns BoundaryCrossing instance
             */
            public static create(properties?: proto.GameEvent.IBoundaryCrossing): proto.GameEvent.BoundaryCrossing;

            /**
             * Encodes the specified BoundaryCrossing message. Does not implicitly {@link proto.GameEvent.BoundaryCrossing.verify|verify} messages.
             * @param message BoundaryCrossing message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IBoundaryCrossing, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified BoundaryCrossing message, length delimited. Does not implicitly {@link proto.GameEvent.BoundaryCrossing.verify|verify} messages.
             * @param message BoundaryCrossing message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IBoundaryCrossing, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a BoundaryCrossing message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns BoundaryCrossing
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.BoundaryCrossing;

            /**
             * Decodes a BoundaryCrossing message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns BoundaryCrossing
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.BoundaryCrossing;

            /**
             * Verifies a BoundaryCrossing message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a BoundaryCrossing message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns BoundaryCrossing
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.BoundaryCrossing;

            /**
             * Creates a plain object from a BoundaryCrossing message. Also converts values to other types if specified.
             * @param message BoundaryCrossing
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.BoundaryCrossing, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this BoundaryCrossing to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for BoundaryCrossing
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Properties of a PenaltyKickFailed. */
        interface IPenaltyKickFailed {

            /** PenaltyKickFailed byTeam */
            byTeam: proto.Team;

            /** PenaltyKickFailed location */
            location?: (proto.IVector2|null);

            /** PenaltyKickFailed reason */
            reason?: (string|null);
        }

        /** Represents a PenaltyKickFailed. */
        class PenaltyKickFailed implements IPenaltyKickFailed {

            /**
             * Constructs a new PenaltyKickFailed.
             * @param [properties] Properties to set
             */
            constructor(properties?: proto.GameEvent.IPenaltyKickFailed);

            /** PenaltyKickFailed byTeam. */
            public byTeam: proto.Team;

            /** PenaltyKickFailed location. */
            public location?: (proto.IVector2|null);

            /** PenaltyKickFailed reason. */
            public reason: string;

            /**
             * Creates a new PenaltyKickFailed instance using the specified properties.
             * @param [properties] Properties to set
             * @returns PenaltyKickFailed instance
             */
            public static create(properties?: proto.GameEvent.IPenaltyKickFailed): proto.GameEvent.PenaltyKickFailed;

            /**
             * Encodes the specified PenaltyKickFailed message. Does not implicitly {@link proto.GameEvent.PenaltyKickFailed.verify|verify} messages.
             * @param message PenaltyKickFailed message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: proto.GameEvent.IPenaltyKickFailed, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified PenaltyKickFailed message, length delimited. Does not implicitly {@link proto.GameEvent.PenaltyKickFailed.verify|verify} messages.
             * @param message PenaltyKickFailed message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: proto.GameEvent.IPenaltyKickFailed, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes a PenaltyKickFailed message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns PenaltyKickFailed
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.GameEvent.PenaltyKickFailed;

            /**
             * Decodes a PenaltyKickFailed message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns PenaltyKickFailed
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.GameEvent.PenaltyKickFailed;

            /**
             * Verifies a PenaltyKickFailed message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates a PenaltyKickFailed message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns PenaltyKickFailed
             */
            public static fromObject(object: { [k: string]: any }): proto.GameEvent.PenaltyKickFailed;

            /**
             * Creates a plain object from a PenaltyKickFailed message. Also converts values to other types if specified.
             * @param message PenaltyKickFailed
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: proto.GameEvent.PenaltyKickFailed, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this PenaltyKickFailed to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for PenaltyKickFailed
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }

        /** Type enum. */
        enum Type {
            UNKNOWN_GAME_EVENT_TYPE = 0,
            BALL_LEFT_FIELD_TOUCH_LINE = 6,
            BALL_LEFT_FIELD_GOAL_LINE = 7,
            AIMLESS_KICK = 11,
            ATTACKER_TOO_CLOSE_TO_DEFENSE_AREA = 19,
            DEFENDER_IN_DEFENSE_AREA = 31,
            BOUNDARY_CROSSING = 41,
            KEEPER_HELD_BALL = 13,
            BOT_DRIBBLED_BALL_TOO_FAR = 17,
            BOT_PUSHED_BOT = 24,
            BOT_HELD_BALL_DELIBERATELY = 26,
            BOT_TIPPED_OVER = 27,
            BOT_DROPPED_PARTS = 47,
            ATTACKER_TOUCHED_BALL_IN_DEFENSE_AREA = 15,
            BOT_KICKED_BALL_TOO_FAST = 18,
            BOT_CRASH_UNIQUE = 22,
            BOT_CRASH_DRAWN = 21,
            DEFENDER_TOO_CLOSE_TO_KICK_POINT = 29,
            BOT_TOO_FAST_IN_STOP = 28,
            BOT_INTERFERED_PLACEMENT = 20,
            POSSIBLE_GOAL = 39,
            GOAL = 8,
            INVALID_GOAL = 42,
            ATTACKER_DOUBLE_TOUCHED_BALL = 14,
            PLACEMENT_SUCCEEDED = 5,
            PENALTY_KICK_FAILED = 43,
            NO_PROGRESS_IN_GAME = 2,
            PLACEMENT_FAILED = 3,
            MULTIPLE_CARDS = 32,
            MULTIPLE_FOULS = 34,
            BOT_SUBSTITUTION = 37,
            TOO_MANY_ROBOTS = 38,
            CHALLENGE_FLAG = 44,
            CHALLENGE_FLAG_HANDLED = 46,
            EMERGENCY_STOP = 45,
            UNSPORTING_BEHAVIOR_MINOR = 35,
            UNSPORTING_BEHAVIOR_MAJOR = 36
        }
    }

    /** Team enum. */
    enum Team {
        UNKNOWN = 0,
        YELLOW = 1,
        BLUE = 2
    }

    /** Properties of a RobotId. */
    interface IRobotId {

        /** RobotId id */
        id?: (number|null);

        /** RobotId team */
        team?: (proto.Team|null);
    }

    /** Represents a RobotId. */
    class RobotId implements IRobotId {

        /**
         * Constructs a new RobotId.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IRobotId);

        /** RobotId id. */
        public id: number;

        /** RobotId team. */
        public team: proto.Team;

        /**
         * Creates a new RobotId instance using the specified properties.
         * @param [properties] Properties to set
         * @returns RobotId instance
         */
        public static create(properties?: proto.IRobotId): proto.RobotId;

        /**
         * Encodes the specified RobotId message. Does not implicitly {@link proto.RobotId.verify|verify} messages.
         * @param message RobotId message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IRobotId, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified RobotId message, length delimited. Does not implicitly {@link proto.RobotId.verify|verify} messages.
         * @param message RobotId message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IRobotId, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a RobotId message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns RobotId
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.RobotId;

        /**
         * Decodes a RobotId message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns RobotId
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.RobotId;

        /**
         * Verifies a RobotId message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a RobotId message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns RobotId
         */
        public static fromObject(object: { [k: string]: any }): proto.RobotId;

        /**
         * Creates a plain object from a RobotId message. Also converts values to other types if specified.
         * @param message RobotId
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.RobotId, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this RobotId to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for RobotId
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Division enum. */
    enum Division {
        DIV_UNKNOWN = 0,
        DIV_A = 1,
        DIV_B = 2
    }

    /** Properties of a Vector2. */
    interface IVector2 {

        /** Vector2 x */
        x: number;

        /** Vector2 y */
        y: number;
    }

    /** Represents a Vector2. */
    class Vector2 implements IVector2 {

        /**
         * Constructs a new Vector2.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IVector2);

        /** Vector2 x. */
        public x: number;

        /** Vector2 y. */
        public y: number;

        /**
         * Creates a new Vector2 instance using the specified properties.
         * @param [properties] Properties to set
         * @returns Vector2 instance
         */
        public static create(properties?: proto.IVector2): proto.Vector2;

        /**
         * Encodes the specified Vector2 message. Does not implicitly {@link proto.Vector2.verify|verify} messages.
         * @param message Vector2 message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IVector2, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified Vector2 message, length delimited. Does not implicitly {@link proto.Vector2.verify|verify} messages.
         * @param message Vector2 message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IVector2, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a Vector2 message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns Vector2
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.Vector2;

        /**
         * Decodes a Vector2 message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns Vector2
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.Vector2;

        /**
         * Verifies a Vector2 message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a Vector2 message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns Vector2
         */
        public static fromObject(object: { [k: string]: any }): proto.Vector2;

        /**
         * Creates a plain object from a Vector2 message. Also converts values to other types if specified.
         * @param message Vector2
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.Vector2, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this Vector2 to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for Vector2
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a Vector3. */
    interface IVector3 {

        /** Vector3 x */
        x: number;

        /** Vector3 y */
        y: number;

        /** Vector3 z */
        z: number;
    }

    /** Represents a Vector3. */
    class Vector3 implements IVector3 {

        /**
         * Constructs a new Vector3.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IVector3);

        /** Vector3 x. */
        public x: number;

        /** Vector3 y. */
        public y: number;

        /** Vector3 z. */
        public z: number;

        /**
         * Creates a new Vector3 instance using the specified properties.
         * @param [properties] Properties to set
         * @returns Vector3 instance
         */
        public static create(properties?: proto.IVector3): proto.Vector3;

        /**
         * Encodes the specified Vector3 message. Does not implicitly {@link proto.Vector3.verify|verify} messages.
         * @param message Vector3 message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IVector3, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified Vector3 message, length delimited. Does not implicitly {@link proto.Vector3.verify|verify} messages.
         * @param message Vector3 message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IVector3, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a Vector3 message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns Vector3
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.Vector3;

        /**
         * Decodes a Vector3 message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns Vector3
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.Vector3;

        /**
         * Verifies a Vector3 message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a Vector3 message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns Vector3
         */
        public static fromObject(object: { [k: string]: any }): proto.Vector3;

        /**
         * Creates a plain object from a Vector3 message. Also converts values to other types if specified.
         * @param message Vector3
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.Vector3, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this Vector3 to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for Vector3
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** RobotTeam enum. */
    enum RobotTeam {
        YELLOW_TEAM = 0,
        BLUE_TEAM = 1
    }

    /** RobotFeedbackSource enum. */
    enum RobotFeedbackSource {
        SIMULATOR = 0,
        BASESTATION = 1
    }

    /** Properties of a RobotFeedback. */
    interface IRobotFeedback {

        /** RobotFeedback id */
        id?: (number|null);

        /** RobotFeedback ballSensorSeesBall */
        ballSensorSeesBall?: (boolean|null);

        /** RobotFeedback ballSensorIsWorking */
        ballSensorIsWorking?: (boolean|null);

        /** RobotFeedback dribblerSeesBall */
        dribblerSeesBall?: (boolean|null);

        /** RobotFeedback estimatedVelocityX */
        estimatedVelocityX?: (number|null);

        /** RobotFeedback estimatedVelocityY */
        estimatedVelocityY?: (number|null);

        /** RobotFeedback estimatedYaw */
        estimatedYaw?: (number|null);

        /** RobotFeedback xsensIsCalibrated */
        xsensIsCalibrated?: (boolean|null);

        /** RobotFeedback capacitorIsCharged */
        capacitorIsCharged?: (boolean|null);

        /** RobotFeedback batteryLevel */
        batteryLevel?: (number|null);
    }

    /** Represents a RobotFeedback. */
    class RobotFeedback implements IRobotFeedback {

        /**
         * Constructs a new RobotFeedback.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IRobotFeedback);

        /** RobotFeedback id. */
        public id: number;

        /** RobotFeedback ballSensorSeesBall. */
        public ballSensorSeesBall: boolean;

        /** RobotFeedback ballSensorIsWorking. */
        public ballSensorIsWorking: boolean;

        /** RobotFeedback dribblerSeesBall. */
        public dribblerSeesBall: boolean;

        /** RobotFeedback estimatedVelocityX. */
        public estimatedVelocityX: number;

        /** RobotFeedback estimatedVelocityY. */
        public estimatedVelocityY: number;

        /** RobotFeedback estimatedYaw. */
        public estimatedYaw: number;

        /** RobotFeedback xsensIsCalibrated. */
        public xsensIsCalibrated: boolean;

        /** RobotFeedback capacitorIsCharged. */
        public capacitorIsCharged: boolean;

        /** RobotFeedback batteryLevel. */
        public batteryLevel: number;

        /**
         * Creates a new RobotFeedback instance using the specified properties.
         * @param [properties] Properties to set
         * @returns RobotFeedback instance
         */
        public static create(properties?: proto.IRobotFeedback): proto.RobotFeedback;

        /**
         * Encodes the specified RobotFeedback message. Does not implicitly {@link proto.RobotFeedback.verify|verify} messages.
         * @param message RobotFeedback message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IRobotFeedback, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified RobotFeedback message, length delimited. Does not implicitly {@link proto.RobotFeedback.verify|verify} messages.
         * @param message RobotFeedback message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IRobotFeedback, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a RobotFeedback message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns RobotFeedback
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.RobotFeedback;

        /**
         * Decodes a RobotFeedback message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns RobotFeedback
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.RobotFeedback;

        /**
         * Verifies a RobotFeedback message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a RobotFeedback message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns RobotFeedback
         */
        public static fromObject(object: { [k: string]: any }): proto.RobotFeedback;

        /**
         * Creates a plain object from a RobotFeedback message. Also converts values to other types if specified.
         * @param message RobotFeedback
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.RobotFeedback, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this RobotFeedback to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for RobotFeedback
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Properties of a RobotsFeedback. */
    interface IRobotsFeedback {

        /** RobotsFeedback team */
        team?: (proto.RobotTeam|null);

        /** RobotsFeedback source */
        source?: (proto.RobotFeedbackSource|null);

        /** RobotsFeedback robotsFeedback */
        robotsFeedback?: (proto.IRobotFeedback[]|null);
    }

    /** Represents a RobotsFeedback. */
    class RobotsFeedback implements IRobotsFeedback {

        /**
         * Constructs a new RobotsFeedback.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IRobotsFeedback);

        /** RobotsFeedback team. */
        public team: proto.RobotTeam;

        /** RobotsFeedback source. */
        public source: proto.RobotFeedbackSource;

        /** RobotsFeedback robotsFeedback. */
        public robotsFeedback: proto.IRobotFeedback[];

        /**
         * Creates a new RobotsFeedback instance using the specified properties.
         * @param [properties] Properties to set
         * @returns RobotsFeedback instance
         */
        public static create(properties?: proto.IRobotsFeedback): proto.RobotsFeedback;

        /**
         * Encodes the specified RobotsFeedback message. Does not implicitly {@link proto.RobotsFeedback.verify|verify} messages.
         * @param message RobotsFeedback message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IRobotsFeedback, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified RobotsFeedback message, length delimited. Does not implicitly {@link proto.RobotsFeedback.verify|verify} messages.
         * @param message RobotsFeedback message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IRobotsFeedback, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a RobotsFeedback message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns RobotsFeedback
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.RobotsFeedback;

        /**
         * Decodes a RobotsFeedback message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns RobotsFeedback
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.RobotsFeedback;

        /**
         * Verifies a RobotsFeedback message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a RobotsFeedback message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns RobotsFeedback
         */
        public static fromObject(object: { [k: string]: any }): proto.RobotsFeedback;

        /**
         * Creates a plain object from a RobotsFeedback message. Also converts values to other types if specified.
         * @param message RobotsFeedback
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.RobotsFeedback, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this RobotsFeedback to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for RobotsFeedback
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    /** Namespace RoboCup2014Legacy. */
    namespace RoboCup2014Legacy {

        /** Namespace Geometry. */
        namespace Geometry {

            /** Properties of a SSL_GeometryFieldSize. */
            interface ISSL_GeometryFieldSize {

                /** SSL_GeometryFieldSize lineWidth */
                lineWidth: number;

                /** SSL_GeometryFieldSize fieldLength */
                fieldLength: number;

                /** SSL_GeometryFieldSize fieldWidth */
                fieldWidth: number;

                /** SSL_GeometryFieldSize boundaryWidth */
                boundaryWidth: number;

                /** SSL_GeometryFieldSize refereeWidth */
                refereeWidth: number;

                /** SSL_GeometryFieldSize goalWidth */
                goalWidth: number;

                /** SSL_GeometryFieldSize goalDepth */
                goalDepth: number;

                /** SSL_GeometryFieldSize goalWallWidth */
                goalWallWidth: number;

                /** SSL_GeometryFieldSize centerCircleRadius */
                centerCircleRadius: number;

                /** SSL_GeometryFieldSize defenseRadius */
                defenseRadius: number;

                /** SSL_GeometryFieldSize defenseStretch */
                defenseStretch: number;

                /** SSL_GeometryFieldSize freeKickFromDefenseDist */
                freeKickFromDefenseDist: number;

                /** SSL_GeometryFieldSize penaltySpotFromFieldLineDist */
                penaltySpotFromFieldLineDist: number;

                /** SSL_GeometryFieldSize penaltyLineFromSpotDist */
                penaltyLineFromSpotDist: number;
            }

            /** Represents a SSL_GeometryFieldSize. */
            class SSL_GeometryFieldSize implements ISSL_GeometryFieldSize {

                /**
                 * Constructs a new SSL_GeometryFieldSize.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: proto.RoboCup2014Legacy.Geometry.ISSL_GeometryFieldSize);

                /** SSL_GeometryFieldSize lineWidth. */
                public lineWidth: number;

                /** SSL_GeometryFieldSize fieldLength. */
                public fieldLength: number;

                /** SSL_GeometryFieldSize fieldWidth. */
                public fieldWidth: number;

                /** SSL_GeometryFieldSize boundaryWidth. */
                public boundaryWidth: number;

                /** SSL_GeometryFieldSize refereeWidth. */
                public refereeWidth: number;

                /** SSL_GeometryFieldSize goalWidth. */
                public goalWidth: number;

                /** SSL_GeometryFieldSize goalDepth. */
                public goalDepth: number;

                /** SSL_GeometryFieldSize goalWallWidth. */
                public goalWallWidth: number;

                /** SSL_GeometryFieldSize centerCircleRadius. */
                public centerCircleRadius: number;

                /** SSL_GeometryFieldSize defenseRadius. */
                public defenseRadius: number;

                /** SSL_GeometryFieldSize defenseStretch. */
                public defenseStretch: number;

                /** SSL_GeometryFieldSize freeKickFromDefenseDist. */
                public freeKickFromDefenseDist: number;

                /** SSL_GeometryFieldSize penaltySpotFromFieldLineDist. */
                public penaltySpotFromFieldLineDist: number;

                /** SSL_GeometryFieldSize penaltyLineFromSpotDist. */
                public penaltyLineFromSpotDist: number;

                /**
                 * Creates a new SSL_GeometryFieldSize instance using the specified properties.
                 * @param [properties] Properties to set
                 * @returns SSL_GeometryFieldSize instance
                 */
                public static create(properties?: proto.RoboCup2014Legacy.Geometry.ISSL_GeometryFieldSize): proto.RoboCup2014Legacy.Geometry.SSL_GeometryFieldSize;

                /**
                 * Encodes the specified SSL_GeometryFieldSize message. Does not implicitly {@link proto.RoboCup2014Legacy.Geometry.SSL_GeometryFieldSize.verify|verify} messages.
                 * @param message SSL_GeometryFieldSize message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: proto.RoboCup2014Legacy.Geometry.ISSL_GeometryFieldSize, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Encodes the specified SSL_GeometryFieldSize message, length delimited. Does not implicitly {@link proto.RoboCup2014Legacy.Geometry.SSL_GeometryFieldSize.verify|verify} messages.
                 * @param message SSL_GeometryFieldSize message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encodeDelimited(message: proto.RoboCup2014Legacy.Geometry.ISSL_GeometryFieldSize, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a SSL_GeometryFieldSize message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns SSL_GeometryFieldSize
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.RoboCup2014Legacy.Geometry.SSL_GeometryFieldSize;

                /**
                 * Decodes a SSL_GeometryFieldSize message from the specified reader or buffer, length delimited.
                 * @param reader Reader or buffer to decode from
                 * @returns SSL_GeometryFieldSize
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.RoboCup2014Legacy.Geometry.SSL_GeometryFieldSize;

                /**
                 * Verifies a SSL_GeometryFieldSize message.
                 * @param message Plain object to verify
                 * @returns `null` if valid, otherwise the reason why it is not
                 */
                public static verify(message: { [k: string]: any }): (string|null);

                /**
                 * Creates a SSL_GeometryFieldSize message from a plain object. Also converts values to their respective internal types.
                 * @param object Plain object
                 * @returns SSL_GeometryFieldSize
                 */
                public static fromObject(object: { [k: string]: any }): proto.RoboCup2014Legacy.Geometry.SSL_GeometryFieldSize;

                /**
                 * Creates a plain object from a SSL_GeometryFieldSize message. Also converts values to other types if specified.
                 * @param message SSL_GeometryFieldSize
                 * @param [options] Conversion options
                 * @returns Plain object
                 */
                public static toObject(message: proto.RoboCup2014Legacy.Geometry.SSL_GeometryFieldSize, options?: $protobuf.IConversionOptions): { [k: string]: any };

                /**
                 * Converts this SSL_GeometryFieldSize to JSON.
                 * @returns JSON object
                 */
                public toJSON(): { [k: string]: any };

                /**
                 * Gets the default type url for SSL_GeometryFieldSize
                 * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
                 * @returns The default type url
                 */
                public static getTypeUrl(typeUrlPrefix?: string): string;
            }

            /** Properties of a SSL_GeometryData. */
            interface ISSL_GeometryData {

                /** SSL_GeometryData field */
                field: proto.RoboCup2014Legacy.Geometry.ISSL_GeometryFieldSize;

                /** SSL_GeometryData calib */
                calib?: (proto.ISSL_GeometryCameraCalibration[]|null);
            }

            /** Represents a SSL_GeometryData. */
            class SSL_GeometryData implements ISSL_GeometryData {

                /**
                 * Constructs a new SSL_GeometryData.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: proto.RoboCup2014Legacy.Geometry.ISSL_GeometryData);

                /** SSL_GeometryData field. */
                public field: proto.RoboCup2014Legacy.Geometry.ISSL_GeometryFieldSize;

                /** SSL_GeometryData calib. */
                public calib: proto.ISSL_GeometryCameraCalibration[];

                /**
                 * Creates a new SSL_GeometryData instance using the specified properties.
                 * @param [properties] Properties to set
                 * @returns SSL_GeometryData instance
                 */
                public static create(properties?: proto.RoboCup2014Legacy.Geometry.ISSL_GeometryData): proto.RoboCup2014Legacy.Geometry.SSL_GeometryData;

                /**
                 * Encodes the specified SSL_GeometryData message. Does not implicitly {@link proto.RoboCup2014Legacy.Geometry.SSL_GeometryData.verify|verify} messages.
                 * @param message SSL_GeometryData message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: proto.RoboCup2014Legacy.Geometry.ISSL_GeometryData, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Encodes the specified SSL_GeometryData message, length delimited. Does not implicitly {@link proto.RoboCup2014Legacy.Geometry.SSL_GeometryData.verify|verify} messages.
                 * @param message SSL_GeometryData message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encodeDelimited(message: proto.RoboCup2014Legacy.Geometry.ISSL_GeometryData, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a SSL_GeometryData message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns SSL_GeometryData
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.RoboCup2014Legacy.Geometry.SSL_GeometryData;

                /**
                 * Decodes a SSL_GeometryData message from the specified reader or buffer, length delimited.
                 * @param reader Reader or buffer to decode from
                 * @returns SSL_GeometryData
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.RoboCup2014Legacy.Geometry.SSL_GeometryData;

                /**
                 * Verifies a SSL_GeometryData message.
                 * @param message Plain object to verify
                 * @returns `null` if valid, otherwise the reason why it is not
                 */
                public static verify(message: { [k: string]: any }): (string|null);

                /**
                 * Creates a SSL_GeometryData message from a plain object. Also converts values to their respective internal types.
                 * @param object Plain object
                 * @returns SSL_GeometryData
                 */
                public static fromObject(object: { [k: string]: any }): proto.RoboCup2014Legacy.Geometry.SSL_GeometryData;

                /**
                 * Creates a plain object from a SSL_GeometryData message. Also converts values to other types if specified.
                 * @param message SSL_GeometryData
                 * @param [options] Conversion options
                 * @returns Plain object
                 */
                public static toObject(message: proto.RoboCup2014Legacy.Geometry.SSL_GeometryData, options?: $protobuf.IConversionOptions): { [k: string]: any };

                /**
                 * Converts this SSL_GeometryData to JSON.
                 * @returns JSON object
                 */
                public toJSON(): { [k: string]: any };

                /**
                 * Gets the default type url for SSL_GeometryData
                 * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
                 * @returns The default type url
                 */
                public static getTypeUrl(typeUrlPrefix?: string): string;
            }
        }

        /** Namespace Wrapper. */
        namespace Wrapper {

            /** Properties of a SSL_WrapperPacket. */
            interface ISSL_WrapperPacket {

                /** SSL_WrapperPacket detection */
                detection?: (proto.ISSL_DetectionFrame|null);

                /** SSL_WrapperPacket geometry */
                geometry?: (proto.RoboCup2014Legacy.Geometry.ISSL_GeometryData|null);
            }

            /** Represents a SSL_WrapperPacket. */
            class SSL_WrapperPacket implements ISSL_WrapperPacket {

                /**
                 * Constructs a new SSL_WrapperPacket.
                 * @param [properties] Properties to set
                 */
                constructor(properties?: proto.RoboCup2014Legacy.Wrapper.ISSL_WrapperPacket);

                /** SSL_WrapperPacket detection. */
                public detection?: (proto.ISSL_DetectionFrame|null);

                /** SSL_WrapperPacket geometry. */
                public geometry?: (proto.RoboCup2014Legacy.Geometry.ISSL_GeometryData|null);

                /**
                 * Creates a new SSL_WrapperPacket instance using the specified properties.
                 * @param [properties] Properties to set
                 * @returns SSL_WrapperPacket instance
                 */
                public static create(properties?: proto.RoboCup2014Legacy.Wrapper.ISSL_WrapperPacket): proto.RoboCup2014Legacy.Wrapper.SSL_WrapperPacket;

                /**
                 * Encodes the specified SSL_WrapperPacket message. Does not implicitly {@link proto.RoboCup2014Legacy.Wrapper.SSL_WrapperPacket.verify|verify} messages.
                 * @param message SSL_WrapperPacket message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encode(message: proto.RoboCup2014Legacy.Wrapper.ISSL_WrapperPacket, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Encodes the specified SSL_WrapperPacket message, length delimited. Does not implicitly {@link proto.RoboCup2014Legacy.Wrapper.SSL_WrapperPacket.verify|verify} messages.
                 * @param message SSL_WrapperPacket message or plain object to encode
                 * @param [writer] Writer to encode to
                 * @returns Writer
                 */
                public static encodeDelimited(message: proto.RoboCup2014Legacy.Wrapper.ISSL_WrapperPacket, writer?: $protobuf.Writer): $protobuf.Writer;

                /**
                 * Decodes a SSL_WrapperPacket message from the specified reader or buffer.
                 * @param reader Reader or buffer to decode from
                 * @param [length] Message length if known beforehand
                 * @returns SSL_WrapperPacket
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.RoboCup2014Legacy.Wrapper.SSL_WrapperPacket;

                /**
                 * Decodes a SSL_WrapperPacket message from the specified reader or buffer, length delimited.
                 * @param reader Reader or buffer to decode from
                 * @returns SSL_WrapperPacket
                 * @throws {Error} If the payload is not a reader or valid buffer
                 * @throws {$protobuf.util.ProtocolError} If required fields are missing
                 */
                public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.RoboCup2014Legacy.Wrapper.SSL_WrapperPacket;

                /**
                 * Verifies a SSL_WrapperPacket message.
                 * @param message Plain object to verify
                 * @returns `null` if valid, otherwise the reason why it is not
                 */
                public static verify(message: { [k: string]: any }): (string|null);

                /**
                 * Creates a SSL_WrapperPacket message from a plain object. Also converts values to their respective internal types.
                 * @param object Plain object
                 * @returns SSL_WrapperPacket
                 */
                public static fromObject(object: { [k: string]: any }): proto.RoboCup2014Legacy.Wrapper.SSL_WrapperPacket;

                /**
                 * Creates a plain object from a SSL_WrapperPacket message. Also converts values to other types if specified.
                 * @param message SSL_WrapperPacket
                 * @param [options] Conversion options
                 * @returns Plain object
                 */
                public static toObject(message: proto.RoboCup2014Legacy.Wrapper.SSL_WrapperPacket, options?: $protobuf.IConversionOptions): { [k: string]: any };

                /**
                 * Converts this SSL_WrapperPacket to JSON.
                 * @returns JSON object
                 */
                public toJSON(): { [k: string]: any };

                /**
                 * Gets the default type url for SSL_WrapperPacket
                 * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
                 * @returns The default type url
                 */
                public static getTypeUrl(typeUrlPrefix?: string): string;
            }
        }
    }

    /** Properties of a RobotCommand. */
    interface IRobotCommand {

        /** RobotCommand id */
        id?: (number|null);

        /** RobotCommand velocityX */
        velocityX?: (number|null);

        /** RobotCommand velocityY */
        velocityY?: (number|null);

        /** RobotCommand yaw */
        yaw?: (number|null);

        /** RobotCommand angularVelocity */
        angularVelocity?: (number|null);

        /** RobotCommand useAngularVelocity */
        useAngularVelocity?: (boolean|null);

        /** RobotCommand cameraYawOfRobot */
        cameraYawOfRobot?: (number|null);

        /** RobotCommand cameraYawOfRobotIsSet */
        cameraYawOfRobotIsSet?: (boolean|null);

        /** RobotCommand kickSpeed */
        kickSpeed?: (number|null);

        /** RobotCommand waitForBall */
        waitForBall?: (boolean|null);

        /** RobotCommand kickAtYaw */
        kickAtYaw?: (boolean|null);

        /** RobotCommand kickType */
        kickType?: (proto.RobotCommand.KickType|null);

        /** RobotCommand dribblerOn */
        dribblerOn?: (boolean|null);

        /** RobotCommand wheelsOff */
        wheelsOff?: (boolean|null);
    }

    /** Represents a RobotCommand. */
    class RobotCommand implements IRobotCommand {

        /**
         * Constructs a new RobotCommand.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IRobotCommand);

        /** RobotCommand id. */
        public id: number;

        /** RobotCommand velocityX. */
        public velocityX: number;

        /** RobotCommand velocityY. */
        public velocityY: number;

        /** RobotCommand yaw. */
        public yaw: number;

        /** RobotCommand angularVelocity. */
        public angularVelocity: number;

        /** RobotCommand useAngularVelocity. */
        public useAngularVelocity: boolean;

        /** RobotCommand cameraYawOfRobot. */
        public cameraYawOfRobot: number;

        /** RobotCommand cameraYawOfRobotIsSet. */
        public cameraYawOfRobotIsSet: boolean;

        /** RobotCommand kickSpeed. */
        public kickSpeed: number;

        /** RobotCommand waitForBall. */
        public waitForBall: boolean;

        /** RobotCommand kickAtYaw. */
        public kickAtYaw: boolean;

        /** RobotCommand kickType. */
        public kickType: proto.RobotCommand.KickType;

        /** RobotCommand dribblerOn. */
        public dribblerOn: boolean;

        /** RobotCommand wheelsOff. */
        public wheelsOff: boolean;

        /**
         * Creates a new RobotCommand instance using the specified properties.
         * @param [properties] Properties to set
         * @returns RobotCommand instance
         */
        public static create(properties?: proto.IRobotCommand): proto.RobotCommand;

        /**
         * Encodes the specified RobotCommand message. Does not implicitly {@link proto.RobotCommand.verify|verify} messages.
         * @param message RobotCommand message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IRobotCommand, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified RobotCommand message, length delimited. Does not implicitly {@link proto.RobotCommand.verify|verify} messages.
         * @param message RobotCommand message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IRobotCommand, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a RobotCommand message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns RobotCommand
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.RobotCommand;

        /**
         * Decodes a RobotCommand message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns RobotCommand
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.RobotCommand;

        /**
         * Verifies a RobotCommand message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a RobotCommand message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns RobotCommand
         */
        public static fromObject(object: { [k: string]: any }): proto.RobotCommand;

        /**
         * Creates a plain object from a RobotCommand message. Also converts values to other types if specified.
         * @param message RobotCommand
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.RobotCommand, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this RobotCommand to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for RobotCommand
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }

    namespace RobotCommand {

        /** KickType enum. */
        enum KickType {
            NO_KICK = 0,
            KICK = 1,
            CHIP = 2
        }
    }

    /** Properties of a RobotCommands. */
    interface IRobotCommands {

        /** RobotCommands robotCommands */
        robotCommands?: (proto.IRobotCommand[]|null);
    }

    /** Represents a RobotCommands. */
    class RobotCommands implements IRobotCommands {

        /**
         * Constructs a new RobotCommands.
         * @param [properties] Properties to set
         */
        constructor(properties?: proto.IRobotCommands);

        /** RobotCommands robotCommands. */
        public robotCommands: proto.IRobotCommand[];

        /**
         * Creates a new RobotCommands instance using the specified properties.
         * @param [properties] Properties to set
         * @returns RobotCommands instance
         */
        public static create(properties?: proto.IRobotCommands): proto.RobotCommands;

        /**
         * Encodes the specified RobotCommands message. Does not implicitly {@link proto.RobotCommands.verify|verify} messages.
         * @param message RobotCommands message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encode(message: proto.IRobotCommands, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Encodes the specified RobotCommands message, length delimited. Does not implicitly {@link proto.RobotCommands.verify|verify} messages.
         * @param message RobotCommands message or plain object to encode
         * @param [writer] Writer to encode to
         * @returns Writer
         */
        public static encodeDelimited(message: proto.IRobotCommands, writer?: $protobuf.Writer): $protobuf.Writer;

        /**
         * Decodes a RobotCommands message from the specified reader or buffer.
         * @param reader Reader or buffer to decode from
         * @param [length] Message length if known beforehand
         * @returns RobotCommands
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): proto.RobotCommands;

        /**
         * Decodes a RobotCommands message from the specified reader or buffer, length delimited.
         * @param reader Reader or buffer to decode from
         * @returns RobotCommands
         * @throws {Error} If the payload is not a reader or valid buffer
         * @throws {$protobuf.util.ProtocolError} If required fields are missing
         */
        public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): proto.RobotCommands;

        /**
         * Verifies a RobotCommands message.
         * @param message Plain object to verify
         * @returns `null` if valid, otherwise the reason why it is not
         */
        public static verify(message: { [k: string]: any }): (string|null);

        /**
         * Creates a RobotCommands message from a plain object. Also converts values to their respective internal types.
         * @param object Plain object
         * @returns RobotCommands
         */
        public static fromObject(object: { [k: string]: any }): proto.RobotCommands;

        /**
         * Creates a plain object from a RobotCommands message. Also converts values to other types if specified.
         * @param message RobotCommands
         * @param [options] Conversion options
         * @returns Plain object
         */
        public static toObject(message: proto.RobotCommands, options?: $protobuf.IConversionOptions): { [k: string]: any };

        /**
         * Converts this RobotCommands to JSON.
         * @returns JSON object
         */
        public toJSON(): { [k: string]: any };

        /**
         * Gets the default type url for RobotCommands
         * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
         * @returns The default type url
         */
        public static getTypeUrl(typeUrlPrefix?: string): string;
    }
}

/** Properties of a TeleportBall. */
export interface ITeleportBall {

    /** TeleportBall x */
    x?: (number|null);

    /** TeleportBall y */
    y?: (number|null);

    /** TeleportBall z */
    z?: (number|null);

    /** TeleportBall vx */
    vx?: (number|null);

    /** TeleportBall vy */
    vy?: (number|null);

    /** TeleportBall vz */
    vz?: (number|null);

    /** TeleportBall teleportSafely */
    teleportSafely?: (boolean|null);

    /** TeleportBall roll */
    roll?: (boolean|null);

    /** TeleportBall byForce */
    byForce?: (boolean|null);
}

/** Represents a TeleportBall. */
export class TeleportBall implements ITeleportBall {

    /**
     * Constructs a new TeleportBall.
     * @param [properties] Properties to set
     */
    constructor(properties?: ITeleportBall);

    /** TeleportBall x. */
    public x: number;

    /** TeleportBall y. */
    public y: number;

    /** TeleportBall z. */
    public z: number;

    /** TeleportBall vx. */
    public vx: number;

    /** TeleportBall vy. */
    public vy: number;

    /** TeleportBall vz. */
    public vz: number;

    /** TeleportBall teleportSafely. */
    public teleportSafely: boolean;

    /** TeleportBall roll. */
    public roll: boolean;

    /** TeleportBall byForce. */
    public byForce: boolean;

    /**
     * Creates a new TeleportBall instance using the specified properties.
     * @param [properties] Properties to set
     * @returns TeleportBall instance
     */
    public static create(properties?: ITeleportBall): TeleportBall;

    /**
     * Encodes the specified TeleportBall message. Does not implicitly {@link TeleportBall.verify|verify} messages.
     * @param message TeleportBall message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: ITeleportBall, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Encodes the specified TeleportBall message, length delimited. Does not implicitly {@link TeleportBall.verify|verify} messages.
     * @param message TeleportBall message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encodeDelimited(message: ITeleportBall, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a TeleportBall message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns TeleportBall
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): TeleportBall;

    /**
     * Decodes a TeleportBall message from the specified reader or buffer, length delimited.
     * @param reader Reader or buffer to decode from
     * @returns TeleportBall
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): TeleportBall;

    /**
     * Verifies a TeleportBall message.
     * @param message Plain object to verify
     * @returns `null` if valid, otherwise the reason why it is not
     */
    public static verify(message: { [k: string]: any }): (string|null);

    /**
     * Creates a TeleportBall message from a plain object. Also converts values to their respective internal types.
     * @param object Plain object
     * @returns TeleportBall
     */
    public static fromObject(object: { [k: string]: any }): TeleportBall;

    /**
     * Creates a plain object from a TeleportBall message. Also converts values to other types if specified.
     * @param message TeleportBall
     * @param [options] Conversion options
     * @returns Plain object
     */
    public static toObject(message: TeleportBall, options?: $protobuf.IConversionOptions): { [k: string]: any };

    /**
     * Converts this TeleportBall to JSON.
     * @returns JSON object
     */
    public toJSON(): { [k: string]: any };

    /**
     * Gets the default type url for TeleportBall
     * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
     * @returns The default type url
     */
    public static getTypeUrl(typeUrlPrefix?: string): string;
}

/** Properties of a TeleportRobot. */
export interface ITeleportRobot {

    /** TeleportRobot id */
    id: IRobotId;

    /** TeleportRobot x */
    x?: (number|null);

    /** TeleportRobot y */
    y?: (number|null);

    /** TeleportRobot orientation */
    orientation?: (number|null);

    /** TeleportRobot vX */
    vX?: (number|null);

    /** TeleportRobot vY */
    vY?: (number|null);

    /** TeleportRobot vAngular */
    vAngular?: (number|null);

    /** TeleportRobot present */
    present?: (boolean|null);

    /** TeleportRobot byForce */
    byForce?: (boolean|null);
}

/** Represents a TeleportRobot. */
export class TeleportRobot implements ITeleportRobot {

    /**
     * Constructs a new TeleportRobot.
     * @param [properties] Properties to set
     */
    constructor(properties?: ITeleportRobot);

    /** TeleportRobot id. */
    public id: IRobotId;

    /** TeleportRobot x. */
    public x: number;

    /** TeleportRobot y. */
    public y: number;

    /** TeleportRobot orientation. */
    public orientation: number;

    /** TeleportRobot vX. */
    public vX: number;

    /** TeleportRobot vY. */
    public vY: number;

    /** TeleportRobot vAngular. */
    public vAngular: number;

    /** TeleportRobot present. */
    public present: boolean;

    /** TeleportRobot byForce. */
    public byForce: boolean;

    /**
     * Creates a new TeleportRobot instance using the specified properties.
     * @param [properties] Properties to set
     * @returns TeleportRobot instance
     */
    public static create(properties?: ITeleportRobot): TeleportRobot;

    /**
     * Encodes the specified TeleportRobot message. Does not implicitly {@link TeleportRobot.verify|verify} messages.
     * @param message TeleportRobot message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: ITeleportRobot, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Encodes the specified TeleportRobot message, length delimited. Does not implicitly {@link TeleportRobot.verify|verify} messages.
     * @param message TeleportRobot message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encodeDelimited(message: ITeleportRobot, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a TeleportRobot message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns TeleportRobot
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): TeleportRobot;

    /**
     * Decodes a TeleportRobot message from the specified reader or buffer, length delimited.
     * @param reader Reader or buffer to decode from
     * @returns TeleportRobot
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): TeleportRobot;

    /**
     * Verifies a TeleportRobot message.
     * @param message Plain object to verify
     * @returns `null` if valid, otherwise the reason why it is not
     */
    public static verify(message: { [k: string]: any }): (string|null);

    /**
     * Creates a TeleportRobot message from a plain object. Also converts values to their respective internal types.
     * @param object Plain object
     * @returns TeleportRobot
     */
    public static fromObject(object: { [k: string]: any }): TeleportRobot;

    /**
     * Creates a plain object from a TeleportRobot message. Also converts values to other types if specified.
     * @param message TeleportRobot
     * @param [options] Conversion options
     * @returns Plain object
     */
    public static toObject(message: TeleportRobot, options?: $protobuf.IConversionOptions): { [k: string]: any };

    /**
     * Converts this TeleportRobot to JSON.
     * @returns JSON object
     */
    public toJSON(): { [k: string]: any };

    /**
     * Gets the default type url for TeleportRobot
     * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
     * @returns The default type url
     */
    public static getTypeUrl(typeUrlPrefix?: string): string;
}

/** Properties of a SimulatorControl. */
export interface ISimulatorControl {

    /** SimulatorControl teleportBall */
    teleportBall?: (ITeleportBall|null);

    /** SimulatorControl teleportRobot */
    teleportRobot?: (ITeleportRobot[]|null);

    /** SimulatorControl simulationSpeed */
    simulationSpeed?: (number|null);
}

/** Represents a SimulatorControl. */
export class SimulatorControl implements ISimulatorControl {

    /**
     * Constructs a new SimulatorControl.
     * @param [properties] Properties to set
     */
    constructor(properties?: ISimulatorControl);

    /** SimulatorControl teleportBall. */
    public teleportBall?: (ITeleportBall|null);

    /** SimulatorControl teleportRobot. */
    public teleportRobot: ITeleportRobot[];

    /** SimulatorControl simulationSpeed. */
    public simulationSpeed: number;

    /**
     * Creates a new SimulatorControl instance using the specified properties.
     * @param [properties] Properties to set
     * @returns SimulatorControl instance
     */
    public static create(properties?: ISimulatorControl): SimulatorControl;

    /**
     * Encodes the specified SimulatorControl message. Does not implicitly {@link SimulatorControl.verify|verify} messages.
     * @param message SimulatorControl message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: ISimulatorControl, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Encodes the specified SimulatorControl message, length delimited. Does not implicitly {@link SimulatorControl.verify|verify} messages.
     * @param message SimulatorControl message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encodeDelimited(message: ISimulatorControl, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a SimulatorControl message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns SimulatorControl
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): SimulatorControl;

    /**
     * Decodes a SimulatorControl message from the specified reader or buffer, length delimited.
     * @param reader Reader or buffer to decode from
     * @returns SimulatorControl
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): SimulatorControl;

    /**
     * Verifies a SimulatorControl message.
     * @param message Plain object to verify
     * @returns `null` if valid, otherwise the reason why it is not
     */
    public static verify(message: { [k: string]: any }): (string|null);

    /**
     * Creates a SimulatorControl message from a plain object. Also converts values to their respective internal types.
     * @param object Plain object
     * @returns SimulatorControl
     */
    public static fromObject(object: { [k: string]: any }): SimulatorControl;

    /**
     * Creates a plain object from a SimulatorControl message. Also converts values to other types if specified.
     * @param message SimulatorControl
     * @param [options] Conversion options
     * @returns Plain object
     */
    public static toObject(message: SimulatorControl, options?: $protobuf.IConversionOptions): { [k: string]: any };

    /**
     * Converts this SimulatorControl to JSON.
     * @returns JSON object
     */
    public toJSON(): { [k: string]: any };

    /**
     * Gets the default type url for SimulatorControl
     * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
     * @returns The default type url
     */
    public static getTypeUrl(typeUrlPrefix?: string): string;
}

/** Properties of a SimulatorCommand. */
export interface ISimulatorCommand {

    /** SimulatorCommand control */
    control?: (ISimulatorControl|null);

    /** SimulatorCommand config */
    config?: (ISimulatorConfig|null);
}

/** Represents a SimulatorCommand. */
export class SimulatorCommand implements ISimulatorCommand {

    /**
     * Constructs a new SimulatorCommand.
     * @param [properties] Properties to set
     */
    constructor(properties?: ISimulatorCommand);

    /** SimulatorCommand control. */
    public control?: (ISimulatorControl|null);

    /** SimulatorCommand config. */
    public config?: (ISimulatorConfig|null);

    /**
     * Creates a new SimulatorCommand instance using the specified properties.
     * @param [properties] Properties to set
     * @returns SimulatorCommand instance
     */
    public static create(properties?: ISimulatorCommand): SimulatorCommand;

    /**
     * Encodes the specified SimulatorCommand message. Does not implicitly {@link SimulatorCommand.verify|verify} messages.
     * @param message SimulatorCommand message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: ISimulatorCommand, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Encodes the specified SimulatorCommand message, length delimited. Does not implicitly {@link SimulatorCommand.verify|verify} messages.
     * @param message SimulatorCommand message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encodeDelimited(message: ISimulatorCommand, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a SimulatorCommand message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns SimulatorCommand
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): SimulatorCommand;

    /**
     * Decodes a SimulatorCommand message from the specified reader or buffer, length delimited.
     * @param reader Reader or buffer to decode from
     * @returns SimulatorCommand
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): SimulatorCommand;

    /**
     * Verifies a SimulatorCommand message.
     * @param message Plain object to verify
     * @returns `null` if valid, otherwise the reason why it is not
     */
    public static verify(message: { [k: string]: any }): (string|null);

    /**
     * Creates a SimulatorCommand message from a plain object. Also converts values to their respective internal types.
     * @param object Plain object
     * @returns SimulatorCommand
     */
    public static fromObject(object: { [k: string]: any }): SimulatorCommand;

    /**
     * Creates a plain object from a SimulatorCommand message. Also converts values to other types if specified.
     * @param message SimulatorCommand
     * @param [options] Conversion options
     * @returns Plain object
     */
    public static toObject(message: SimulatorCommand, options?: $protobuf.IConversionOptions): { [k: string]: any };

    /**
     * Converts this SimulatorCommand to JSON.
     * @returns JSON object
     */
    public toJSON(): { [k: string]: any };

    /**
     * Gets the default type url for SimulatorCommand
     * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
     * @returns The default type url
     */
    public static getTypeUrl(typeUrlPrefix?: string): string;
}

/** Properties of a SimulatorResponse. */
export interface ISimulatorResponse {

    /** SimulatorResponse errors */
    errors?: (ISimulatorError[]|null);
}

/** Represents a SimulatorResponse. */
export class SimulatorResponse implements ISimulatorResponse {

    /**
     * Constructs a new SimulatorResponse.
     * @param [properties] Properties to set
     */
    constructor(properties?: ISimulatorResponse);

    /** SimulatorResponse errors. */
    public errors: ISimulatorError[];

    /**
     * Creates a new SimulatorResponse instance using the specified properties.
     * @param [properties] Properties to set
     * @returns SimulatorResponse instance
     */
    public static create(properties?: ISimulatorResponse): SimulatorResponse;

    /**
     * Encodes the specified SimulatorResponse message. Does not implicitly {@link SimulatorResponse.verify|verify} messages.
     * @param message SimulatorResponse message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: ISimulatorResponse, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Encodes the specified SimulatorResponse message, length delimited. Does not implicitly {@link SimulatorResponse.verify|verify} messages.
     * @param message SimulatorResponse message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encodeDelimited(message: ISimulatorResponse, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a SimulatorResponse message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns SimulatorResponse
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): SimulatorResponse;

    /**
     * Decodes a SimulatorResponse message from the specified reader or buffer, length delimited.
     * @param reader Reader or buffer to decode from
     * @returns SimulatorResponse
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): SimulatorResponse;

    /**
     * Verifies a SimulatorResponse message.
     * @param message Plain object to verify
     * @returns `null` if valid, otherwise the reason why it is not
     */
    public static verify(message: { [k: string]: any }): (string|null);

    /**
     * Creates a SimulatorResponse message from a plain object. Also converts values to their respective internal types.
     * @param object Plain object
     * @returns SimulatorResponse
     */
    public static fromObject(object: { [k: string]: any }): SimulatorResponse;

    /**
     * Creates a plain object from a SimulatorResponse message. Also converts values to other types if specified.
     * @param message SimulatorResponse
     * @param [options] Conversion options
     * @returns Plain object
     */
    public static toObject(message: SimulatorResponse, options?: $protobuf.IConversionOptions): { [k: string]: any };

    /**
     * Converts this SimulatorResponse to JSON.
     * @returns JSON object
     */
    public toJSON(): { [k: string]: any };

    /**
     * Gets the default type url for SimulatorResponse
     * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
     * @returns The default type url
     */
    public static getTypeUrl(typeUrlPrefix?: string): string;
}

/** Team enum. */
export enum Team {
    UNKNOWN = 0,
    YELLOW = 1,
    BLUE = 2
}

/** Represents a RobotId. */
export class RobotId implements IRobotId {

    /**
     * Constructs a new RobotId.
     * @param [properties] Properties to set
     */
    constructor(properties?: IRobotId);

    /** RobotId id. */
    public id: number;

    /** RobotId team. */
    public team: Team;

    /**
     * Creates a new RobotId instance using the specified properties.
     * @param [properties] Properties to set
     * @returns RobotId instance
     */
    public static create(properties?: IRobotId): RobotId;

    /**
     * Encodes the specified RobotId message. Does not implicitly {@link RobotId.verify|verify} messages.
     * @param message RobotId message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: IRobotId, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Encodes the specified RobotId message, length delimited. Does not implicitly {@link RobotId.verify|verify} messages.
     * @param message RobotId message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encodeDelimited(message: IRobotId, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a RobotId message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns RobotId
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): RobotId;

    /**
     * Decodes a RobotId message from the specified reader or buffer, length delimited.
     * @param reader Reader or buffer to decode from
     * @returns RobotId
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): RobotId;

    /**
     * Verifies a RobotId message.
     * @param message Plain object to verify
     * @returns `null` if valid, otherwise the reason why it is not
     */
    public static verify(message: { [k: string]: any }): (string|null);

    /**
     * Creates a RobotId message from a plain object. Also converts values to their respective internal types.
     * @param object Plain object
     * @returns RobotId
     */
    public static fromObject(object: { [k: string]: any }): RobotId;

    /**
     * Creates a plain object from a RobotId message. Also converts values to other types if specified.
     * @param message RobotId
     * @param [options] Conversion options
     * @returns Plain object
     */
    public static toObject(message: RobotId, options?: $protobuf.IConversionOptions): { [k: string]: any };

    /**
     * Converts this RobotId to JSON.
     * @returns JSON object
     */
    public toJSON(): { [k: string]: any };

    /**
     * Gets the default type url for RobotId
     * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
     * @returns The default type url
     */
    public static getTypeUrl(typeUrlPrefix?: string): string;
}

/** Division enum. */
export enum Division {
    DIV_UNKNOWN = 0,
    DIV_A = 1,
    DIV_B = 2
}

/** Represents a RobotLimits. */
export class RobotLimits implements IRobotLimits {

    /**
     * Constructs a new RobotLimits.
     * @param [properties] Properties to set
     */
    constructor(properties?: IRobotLimits);

    /** RobotLimits accSpeedupAbsoluteMax. */
    public accSpeedupAbsoluteMax: number;

    /** RobotLimits accSpeedupAngularMax. */
    public accSpeedupAngularMax: number;

    /** RobotLimits accBrakeAbsoluteMax. */
    public accBrakeAbsoluteMax: number;

    /** RobotLimits accBrakeAngularMax. */
    public accBrakeAngularMax: number;

    /** RobotLimits velAbsoluteMax. */
    public velAbsoluteMax: number;

    /** RobotLimits velAngularMax. */
    public velAngularMax: number;

    /**
     * Creates a new RobotLimits instance using the specified properties.
     * @param [properties] Properties to set
     * @returns RobotLimits instance
     */
    public static create(properties?: IRobotLimits): RobotLimits;

    /**
     * Encodes the specified RobotLimits message. Does not implicitly {@link RobotLimits.verify|verify} messages.
     * @param message RobotLimits message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: IRobotLimits, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Encodes the specified RobotLimits message, length delimited. Does not implicitly {@link RobotLimits.verify|verify} messages.
     * @param message RobotLimits message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encodeDelimited(message: IRobotLimits, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a RobotLimits message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns RobotLimits
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): RobotLimits;

    /**
     * Decodes a RobotLimits message from the specified reader or buffer, length delimited.
     * @param reader Reader or buffer to decode from
     * @returns RobotLimits
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): RobotLimits;

    /**
     * Verifies a RobotLimits message.
     * @param message Plain object to verify
     * @returns `null` if valid, otherwise the reason why it is not
     */
    public static verify(message: { [k: string]: any }): (string|null);

    /**
     * Creates a RobotLimits message from a plain object. Also converts values to their respective internal types.
     * @param object Plain object
     * @returns RobotLimits
     */
    public static fromObject(object: { [k: string]: any }): RobotLimits;

    /**
     * Creates a plain object from a RobotLimits message. Also converts values to other types if specified.
     * @param message RobotLimits
     * @param [options] Conversion options
     * @returns Plain object
     */
    public static toObject(message: RobotLimits, options?: $protobuf.IConversionOptions): { [k: string]: any };

    /**
     * Converts this RobotLimits to JSON.
     * @returns JSON object
     */
    public toJSON(): { [k: string]: any };

    /**
     * Gets the default type url for RobotLimits
     * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
     * @returns The default type url
     */
    public static getTypeUrl(typeUrlPrefix?: string): string;
}

/** Represents a RobotWheelAngles. */
export class RobotWheelAngles implements IRobotWheelAngles {

    /**
     * Constructs a new RobotWheelAngles.
     * @param [properties] Properties to set
     */
    constructor(properties?: IRobotWheelAngles);

    /** RobotWheelAngles frontRight. */
    public frontRight: number;

    /** RobotWheelAngles backRight. */
    public backRight: number;

    /** RobotWheelAngles backLeft. */
    public backLeft: number;

    /** RobotWheelAngles frontLeft. */
    public frontLeft: number;

    /**
     * Creates a new RobotWheelAngles instance using the specified properties.
     * @param [properties] Properties to set
     * @returns RobotWheelAngles instance
     */
    public static create(properties?: IRobotWheelAngles): RobotWheelAngles;

    /**
     * Encodes the specified RobotWheelAngles message. Does not implicitly {@link RobotWheelAngles.verify|verify} messages.
     * @param message RobotWheelAngles message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: IRobotWheelAngles, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Encodes the specified RobotWheelAngles message, length delimited. Does not implicitly {@link RobotWheelAngles.verify|verify} messages.
     * @param message RobotWheelAngles message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encodeDelimited(message: IRobotWheelAngles, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a RobotWheelAngles message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns RobotWheelAngles
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): RobotWheelAngles;

    /**
     * Decodes a RobotWheelAngles message from the specified reader or buffer, length delimited.
     * @param reader Reader or buffer to decode from
     * @returns RobotWheelAngles
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): RobotWheelAngles;

    /**
     * Verifies a RobotWheelAngles message.
     * @param message Plain object to verify
     * @returns `null` if valid, otherwise the reason why it is not
     */
    public static verify(message: { [k: string]: any }): (string|null);

    /**
     * Creates a RobotWheelAngles message from a plain object. Also converts values to their respective internal types.
     * @param object Plain object
     * @returns RobotWheelAngles
     */
    public static fromObject(object: { [k: string]: any }): RobotWheelAngles;

    /**
     * Creates a plain object from a RobotWheelAngles message. Also converts values to other types if specified.
     * @param message RobotWheelAngles
     * @param [options] Conversion options
     * @returns Plain object
     */
    public static toObject(message: RobotWheelAngles, options?: $protobuf.IConversionOptions): { [k: string]: any };

    /**
     * Converts this RobotWheelAngles to JSON.
     * @returns JSON object
     */
    public toJSON(): { [k: string]: any };

    /**
     * Gets the default type url for RobotWheelAngles
     * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
     * @returns The default type url
     */
    public static getTypeUrl(typeUrlPrefix?: string): string;
}

/** Represents a RobotSpecs. */
export class RobotSpecs implements IRobotSpecs {

    /**
     * Constructs a new RobotSpecs.
     * @param [properties] Properties to set
     */
    constructor(properties?: IRobotSpecs);

    /** RobotSpecs id. */
    public id: IRobotId;

    /** RobotSpecs radius. */
    public radius: number;

    /** RobotSpecs height. */
    public height: number;

    /** RobotSpecs mass. */
    public mass: number;

    /** RobotSpecs maxLinearKickSpeed. */
    public maxLinearKickSpeed: number;

    /** RobotSpecs maxChipKickSpeed. */
    public maxChipKickSpeed: number;

    /** RobotSpecs centerToDribbler. */
    public centerToDribbler: number;

    /** RobotSpecs limits. */
    public limits?: (IRobotLimits|null);

    /** RobotSpecs wheelAngles. */
    public wheelAngles?: (IRobotWheelAngles|null);

    /** RobotSpecs custom. */
    public custom: google.protobuf.IAny[];

    /**
     * Creates a new RobotSpecs instance using the specified properties.
     * @param [properties] Properties to set
     * @returns RobotSpecs instance
     */
    public static create(properties?: IRobotSpecs): RobotSpecs;

    /**
     * Encodes the specified RobotSpecs message. Does not implicitly {@link RobotSpecs.verify|verify} messages.
     * @param message RobotSpecs message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: IRobotSpecs, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Encodes the specified RobotSpecs message, length delimited. Does not implicitly {@link RobotSpecs.verify|verify} messages.
     * @param message RobotSpecs message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encodeDelimited(message: IRobotSpecs, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a RobotSpecs message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns RobotSpecs
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): RobotSpecs;

    /**
     * Decodes a RobotSpecs message from the specified reader or buffer, length delimited.
     * @param reader Reader or buffer to decode from
     * @returns RobotSpecs
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): RobotSpecs;

    /**
     * Verifies a RobotSpecs message.
     * @param message Plain object to verify
     * @returns `null` if valid, otherwise the reason why it is not
     */
    public static verify(message: { [k: string]: any }): (string|null);

    /**
     * Creates a RobotSpecs message from a plain object. Also converts values to their respective internal types.
     * @param object Plain object
     * @returns RobotSpecs
     */
    public static fromObject(object: { [k: string]: any }): RobotSpecs;

    /**
     * Creates a plain object from a RobotSpecs message. Also converts values to other types if specified.
     * @param message RobotSpecs
     * @param [options] Conversion options
     * @returns Plain object
     */
    public static toObject(message: RobotSpecs, options?: $protobuf.IConversionOptions): { [k: string]: any };

    /**
     * Converts this RobotSpecs to JSON.
     * @returns JSON object
     */
    public toJSON(): { [k: string]: any };

    /**
     * Gets the default type url for RobotSpecs
     * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
     * @returns The default type url
     */
    public static getTypeUrl(typeUrlPrefix?: string): string;
}

/** Represents a RealismConfig. */
export class RealismConfig implements IRealismConfig {

    /**
     * Constructs a new RealismConfig.
     * @param [properties] Properties to set
     */
    constructor(properties?: IRealismConfig);

    /** RealismConfig custom. */
    public custom: google.protobuf.IAny[];

    /**
     * Creates a new RealismConfig instance using the specified properties.
     * @param [properties] Properties to set
     * @returns RealismConfig instance
     */
    public static create(properties?: IRealismConfig): RealismConfig;

    /**
     * Encodes the specified RealismConfig message. Does not implicitly {@link RealismConfig.verify|verify} messages.
     * @param message RealismConfig message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: IRealismConfig, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Encodes the specified RealismConfig message, length delimited. Does not implicitly {@link RealismConfig.verify|verify} messages.
     * @param message RealismConfig message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encodeDelimited(message: IRealismConfig, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a RealismConfig message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns RealismConfig
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): RealismConfig;

    /**
     * Decodes a RealismConfig message from the specified reader or buffer, length delimited.
     * @param reader Reader or buffer to decode from
     * @returns RealismConfig
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): RealismConfig;

    /**
     * Verifies a RealismConfig message.
     * @param message Plain object to verify
     * @returns `null` if valid, otherwise the reason why it is not
     */
    public static verify(message: { [k: string]: any }): (string|null);

    /**
     * Creates a RealismConfig message from a plain object. Also converts values to their respective internal types.
     * @param object Plain object
     * @returns RealismConfig
     */
    public static fromObject(object: { [k: string]: any }): RealismConfig;

    /**
     * Creates a plain object from a RealismConfig message. Also converts values to other types if specified.
     * @param message RealismConfig
     * @param [options] Conversion options
     * @returns Plain object
     */
    public static toObject(message: RealismConfig, options?: $protobuf.IConversionOptions): { [k: string]: any };

    /**
     * Converts this RealismConfig to JSON.
     * @returns JSON object
     */
    public toJSON(): { [k: string]: any };

    /**
     * Gets the default type url for RealismConfig
     * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
     * @returns The default type url
     */
    public static getTypeUrl(typeUrlPrefix?: string): string;
}

/** Represents a SimulatorConfig. */
export class SimulatorConfig implements ISimulatorConfig {

    /**
     * Constructs a new SimulatorConfig.
     * @param [properties] Properties to set
     */
    constructor(properties?: ISimulatorConfig);

    /** SimulatorConfig geometry. */
    public geometry?: (ISSL_GeometryData|null);

    /** SimulatorConfig robotSpecs. */
    public robotSpecs: IRobotSpecs[];

    /** SimulatorConfig realismConfig. */
    public realismConfig?: (IRealismConfig|null);

    /** SimulatorConfig visionPort. */
    public visionPort: number;

    /**
     * Creates a new SimulatorConfig instance using the specified properties.
     * @param [properties] Properties to set
     * @returns SimulatorConfig instance
     */
    public static create(properties?: ISimulatorConfig): SimulatorConfig;

    /**
     * Encodes the specified SimulatorConfig message. Does not implicitly {@link SimulatorConfig.verify|verify} messages.
     * @param message SimulatorConfig message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: ISimulatorConfig, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Encodes the specified SimulatorConfig message, length delimited. Does not implicitly {@link SimulatorConfig.verify|verify} messages.
     * @param message SimulatorConfig message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encodeDelimited(message: ISimulatorConfig, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a SimulatorConfig message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns SimulatorConfig
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): SimulatorConfig;

    /**
     * Decodes a SimulatorConfig message from the specified reader or buffer, length delimited.
     * @param reader Reader or buffer to decode from
     * @returns SimulatorConfig
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): SimulatorConfig;

    /**
     * Verifies a SimulatorConfig message.
     * @param message Plain object to verify
     * @returns `null` if valid, otherwise the reason why it is not
     */
    public static verify(message: { [k: string]: any }): (string|null);

    /**
     * Creates a SimulatorConfig message from a plain object. Also converts values to their respective internal types.
     * @param object Plain object
     * @returns SimulatorConfig
     */
    public static fromObject(object: { [k: string]: any }): SimulatorConfig;

    /**
     * Creates a plain object from a SimulatorConfig message. Also converts values to other types if specified.
     * @param message SimulatorConfig
     * @param [options] Conversion options
     * @returns Plain object
     */
    public static toObject(message: SimulatorConfig, options?: $protobuf.IConversionOptions): { [k: string]: any };

    /**
     * Converts this SimulatorConfig to JSON.
     * @returns JSON object
     */
    public toJSON(): { [k: string]: any };

    /**
     * Gets the default type url for SimulatorConfig
     * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
     * @returns The default type url
     */
    public static getTypeUrl(typeUrlPrefix?: string): string;
}

/** Represents a Vector2f. */
export class Vector2f implements IVector2f {

    /**
     * Constructs a new Vector2f.
     * @param [properties] Properties to set
     */
    constructor(properties?: IVector2f);

    /** Vector2f x. */
    public x: number;

    /** Vector2f y. */
    public y: number;

    /**
     * Creates a new Vector2f instance using the specified properties.
     * @param [properties] Properties to set
     * @returns Vector2f instance
     */
    public static create(properties?: IVector2f): Vector2f;

    /**
     * Encodes the specified Vector2f message. Does not implicitly {@link Vector2f.verify|verify} messages.
     * @param message Vector2f message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: IVector2f, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Encodes the specified Vector2f message, length delimited. Does not implicitly {@link Vector2f.verify|verify} messages.
     * @param message Vector2f message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encodeDelimited(message: IVector2f, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a Vector2f message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns Vector2f
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): Vector2f;

    /**
     * Decodes a Vector2f message from the specified reader or buffer, length delimited.
     * @param reader Reader or buffer to decode from
     * @returns Vector2f
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): Vector2f;

    /**
     * Verifies a Vector2f message.
     * @param message Plain object to verify
     * @returns `null` if valid, otherwise the reason why it is not
     */
    public static verify(message: { [k: string]: any }): (string|null);

    /**
     * Creates a Vector2f message from a plain object. Also converts values to their respective internal types.
     * @param object Plain object
     * @returns Vector2f
     */
    public static fromObject(object: { [k: string]: any }): Vector2f;

    /**
     * Creates a plain object from a Vector2f message. Also converts values to other types if specified.
     * @param message Vector2f
     * @param [options] Conversion options
     * @returns Plain object
     */
    public static toObject(message: Vector2f, options?: $protobuf.IConversionOptions): { [k: string]: any };

    /**
     * Converts this Vector2f to JSON.
     * @returns JSON object
     */
    public toJSON(): { [k: string]: any };

    /**
     * Gets the default type url for Vector2f
     * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
     * @returns The default type url
     */
    public static getTypeUrl(typeUrlPrefix?: string): string;
}

/** Represents a SSL_FieldLineSegment. */
export class SSL_FieldLineSegment implements ISSL_FieldLineSegment {

    /**
     * Constructs a new SSL_FieldLineSegment.
     * @param [properties] Properties to set
     */
    constructor(properties?: ISSL_FieldLineSegment);

    /** SSL_FieldLineSegment name. */
    public name: string;

    /** SSL_FieldLineSegment p1. */
    public p1: IVector2f;

    /** SSL_FieldLineSegment p2. */
    public p2: IVector2f;

    /** SSL_FieldLineSegment thickness. */
    public thickness: number;

    /** SSL_FieldLineSegment type. */
    public type: SSL_FieldShapeType;

    /**
     * Creates a new SSL_FieldLineSegment instance using the specified properties.
     * @param [properties] Properties to set
     * @returns SSL_FieldLineSegment instance
     */
    public static create(properties?: ISSL_FieldLineSegment): SSL_FieldLineSegment;

    /**
     * Encodes the specified SSL_FieldLineSegment message. Does not implicitly {@link SSL_FieldLineSegment.verify|verify} messages.
     * @param message SSL_FieldLineSegment message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: ISSL_FieldLineSegment, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Encodes the specified SSL_FieldLineSegment message, length delimited. Does not implicitly {@link SSL_FieldLineSegment.verify|verify} messages.
     * @param message SSL_FieldLineSegment message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encodeDelimited(message: ISSL_FieldLineSegment, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a SSL_FieldLineSegment message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns SSL_FieldLineSegment
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): SSL_FieldLineSegment;

    /**
     * Decodes a SSL_FieldLineSegment message from the specified reader or buffer, length delimited.
     * @param reader Reader or buffer to decode from
     * @returns SSL_FieldLineSegment
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): SSL_FieldLineSegment;

    /**
     * Verifies a SSL_FieldLineSegment message.
     * @param message Plain object to verify
     * @returns `null` if valid, otherwise the reason why it is not
     */
    public static verify(message: { [k: string]: any }): (string|null);

    /**
     * Creates a SSL_FieldLineSegment message from a plain object. Also converts values to their respective internal types.
     * @param object Plain object
     * @returns SSL_FieldLineSegment
     */
    public static fromObject(object: { [k: string]: any }): SSL_FieldLineSegment;

    /**
     * Creates a plain object from a SSL_FieldLineSegment message. Also converts values to other types if specified.
     * @param message SSL_FieldLineSegment
     * @param [options] Conversion options
     * @returns Plain object
     */
    public static toObject(message: SSL_FieldLineSegment, options?: $protobuf.IConversionOptions): { [k: string]: any };

    /**
     * Converts this SSL_FieldLineSegment to JSON.
     * @returns JSON object
     */
    public toJSON(): { [k: string]: any };

    /**
     * Gets the default type url for SSL_FieldLineSegment
     * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
     * @returns The default type url
     */
    public static getTypeUrl(typeUrlPrefix?: string): string;
}

/** Represents a SSL_FieldCircularArc. */
export class SSL_FieldCircularArc implements ISSL_FieldCircularArc {

    /**
     * Constructs a new SSL_FieldCircularArc.
     * @param [properties] Properties to set
     */
    constructor(properties?: ISSL_FieldCircularArc);

    /** SSL_FieldCircularArc name. */
    public name: string;

    /** SSL_FieldCircularArc center. */
    public center: IVector2f;

    /** SSL_FieldCircularArc radius. */
    public radius: number;

    /** SSL_FieldCircularArc a1. */
    public a1: number;

    /** SSL_FieldCircularArc a2. */
    public a2: number;

    /** SSL_FieldCircularArc thickness. */
    public thickness: number;

    /** SSL_FieldCircularArc type. */
    public type: SSL_FieldShapeType;

    /**
     * Creates a new SSL_FieldCircularArc instance using the specified properties.
     * @param [properties] Properties to set
     * @returns SSL_FieldCircularArc instance
     */
    public static create(properties?: ISSL_FieldCircularArc): SSL_FieldCircularArc;

    /**
     * Encodes the specified SSL_FieldCircularArc message. Does not implicitly {@link SSL_FieldCircularArc.verify|verify} messages.
     * @param message SSL_FieldCircularArc message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: ISSL_FieldCircularArc, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Encodes the specified SSL_FieldCircularArc message, length delimited. Does not implicitly {@link SSL_FieldCircularArc.verify|verify} messages.
     * @param message SSL_FieldCircularArc message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encodeDelimited(message: ISSL_FieldCircularArc, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a SSL_FieldCircularArc message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns SSL_FieldCircularArc
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): SSL_FieldCircularArc;

    /**
     * Decodes a SSL_FieldCircularArc message from the specified reader or buffer, length delimited.
     * @param reader Reader or buffer to decode from
     * @returns SSL_FieldCircularArc
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): SSL_FieldCircularArc;

    /**
     * Verifies a SSL_FieldCircularArc message.
     * @param message Plain object to verify
     * @returns `null` if valid, otherwise the reason why it is not
     */
    public static verify(message: { [k: string]: any }): (string|null);

    /**
     * Creates a SSL_FieldCircularArc message from a plain object. Also converts values to their respective internal types.
     * @param object Plain object
     * @returns SSL_FieldCircularArc
     */
    public static fromObject(object: { [k: string]: any }): SSL_FieldCircularArc;

    /**
     * Creates a plain object from a SSL_FieldCircularArc message. Also converts values to other types if specified.
     * @param message SSL_FieldCircularArc
     * @param [options] Conversion options
     * @returns Plain object
     */
    public static toObject(message: SSL_FieldCircularArc, options?: $protobuf.IConversionOptions): { [k: string]: any };

    /**
     * Converts this SSL_FieldCircularArc to JSON.
     * @returns JSON object
     */
    public toJSON(): { [k: string]: any };

    /**
     * Gets the default type url for SSL_FieldCircularArc
     * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
     * @returns The default type url
     */
    public static getTypeUrl(typeUrlPrefix?: string): string;
}

/** Represents a SSL_GeometryFieldSize. */
export class SSL_GeometryFieldSize implements ISSL_GeometryFieldSize {

    /**
     * Constructs a new SSL_GeometryFieldSize.
     * @param [properties] Properties to set
     */
    constructor(properties?: ISSL_GeometryFieldSize);

    /** SSL_GeometryFieldSize fieldLength. */
    public fieldLength: number;

    /** SSL_GeometryFieldSize fieldWidth. */
    public fieldWidth: number;

    /** SSL_GeometryFieldSize goalWidth. */
    public goalWidth: number;

    /** SSL_GeometryFieldSize goalDepth. */
    public goalDepth: number;

    /** SSL_GeometryFieldSize boundaryWidth. */
    public boundaryWidth: number;

    /** SSL_GeometryFieldSize fieldLines. */
    public fieldLines: ISSL_FieldLineSegment[];

    /** SSL_GeometryFieldSize fieldArcs. */
    public fieldArcs: ISSL_FieldCircularArc[];

    /** SSL_GeometryFieldSize penaltyAreaDepth. */
    public penaltyAreaDepth: number;

    /** SSL_GeometryFieldSize penaltyAreaWidth. */
    public penaltyAreaWidth: number;

    /** SSL_GeometryFieldSize centerCircleRadius. */
    public centerCircleRadius: number;

    /** SSL_GeometryFieldSize lineThickness. */
    public lineThickness: number;

    /** SSL_GeometryFieldSize goalCenterToPenaltyMark. */
    public goalCenterToPenaltyMark: number;

    /** SSL_GeometryFieldSize goalHeight. */
    public goalHeight: number;

    /** SSL_GeometryFieldSize ballRadius. */
    public ballRadius: number;

    /** SSL_GeometryFieldSize maxRobotRadius. */
    public maxRobotRadius: number;

    /**
     * Creates a new SSL_GeometryFieldSize instance using the specified properties.
     * @param [properties] Properties to set
     * @returns SSL_GeometryFieldSize instance
     */
    public static create(properties?: ISSL_GeometryFieldSize): SSL_GeometryFieldSize;

    /**
     * Encodes the specified SSL_GeometryFieldSize message. Does not implicitly {@link SSL_GeometryFieldSize.verify|verify} messages.
     * @param message SSL_GeometryFieldSize message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: ISSL_GeometryFieldSize, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Encodes the specified SSL_GeometryFieldSize message, length delimited. Does not implicitly {@link SSL_GeometryFieldSize.verify|verify} messages.
     * @param message SSL_GeometryFieldSize message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encodeDelimited(message: ISSL_GeometryFieldSize, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a SSL_GeometryFieldSize message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns SSL_GeometryFieldSize
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): SSL_GeometryFieldSize;

    /**
     * Decodes a SSL_GeometryFieldSize message from the specified reader or buffer, length delimited.
     * @param reader Reader or buffer to decode from
     * @returns SSL_GeometryFieldSize
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): SSL_GeometryFieldSize;

    /**
     * Verifies a SSL_GeometryFieldSize message.
     * @param message Plain object to verify
     * @returns `null` if valid, otherwise the reason why it is not
     */
    public static verify(message: { [k: string]: any }): (string|null);

    /**
     * Creates a SSL_GeometryFieldSize message from a plain object. Also converts values to their respective internal types.
     * @param object Plain object
     * @returns SSL_GeometryFieldSize
     */
    public static fromObject(object: { [k: string]: any }): SSL_GeometryFieldSize;

    /**
     * Creates a plain object from a SSL_GeometryFieldSize message. Also converts values to other types if specified.
     * @param message SSL_GeometryFieldSize
     * @param [options] Conversion options
     * @returns Plain object
     */
    public static toObject(message: SSL_GeometryFieldSize, options?: $protobuf.IConversionOptions): { [k: string]: any };

    /**
     * Converts this SSL_GeometryFieldSize to JSON.
     * @returns JSON object
     */
    public toJSON(): { [k: string]: any };

    /**
     * Gets the default type url for SSL_GeometryFieldSize
     * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
     * @returns The default type url
     */
    public static getTypeUrl(typeUrlPrefix?: string): string;
}

/** Represents a SSL_GeometryCameraCalibration. */
export class SSL_GeometryCameraCalibration implements ISSL_GeometryCameraCalibration {

    /**
     * Constructs a new SSL_GeometryCameraCalibration.
     * @param [properties] Properties to set
     */
    constructor(properties?: ISSL_GeometryCameraCalibration);

    /** SSL_GeometryCameraCalibration cameraId. */
    public cameraId: number;

    /** SSL_GeometryCameraCalibration focalLength. */
    public focalLength: number;

    /** SSL_GeometryCameraCalibration principalPointX. */
    public principalPointX: number;

    /** SSL_GeometryCameraCalibration principalPointY. */
    public principalPointY: number;

    /** SSL_GeometryCameraCalibration distortion. */
    public distortion: number;

    /** SSL_GeometryCameraCalibration q0. */
    public q0: number;

    /** SSL_GeometryCameraCalibration q1. */
    public q1: number;

    /** SSL_GeometryCameraCalibration q2. */
    public q2: number;

    /** SSL_GeometryCameraCalibration q3. */
    public q3: number;

    /** SSL_GeometryCameraCalibration tx. */
    public tx: number;

    /** SSL_GeometryCameraCalibration ty. */
    public ty: number;

    /** SSL_GeometryCameraCalibration tz. */
    public tz: number;

    /** SSL_GeometryCameraCalibration derivedCameraWorldTx. */
    public derivedCameraWorldTx: number;

    /** SSL_GeometryCameraCalibration derivedCameraWorldTy. */
    public derivedCameraWorldTy: number;

    /** SSL_GeometryCameraCalibration derivedCameraWorldTz. */
    public derivedCameraWorldTz: number;

    /** SSL_GeometryCameraCalibration pixelImageWidth. */
    public pixelImageWidth: number;

    /** SSL_GeometryCameraCalibration pixelImageHeight. */
    public pixelImageHeight: number;

    /**
     * Creates a new SSL_GeometryCameraCalibration instance using the specified properties.
     * @param [properties] Properties to set
     * @returns SSL_GeometryCameraCalibration instance
     */
    public static create(properties?: ISSL_GeometryCameraCalibration): SSL_GeometryCameraCalibration;

    /**
     * Encodes the specified SSL_GeometryCameraCalibration message. Does not implicitly {@link SSL_GeometryCameraCalibration.verify|verify} messages.
     * @param message SSL_GeometryCameraCalibration message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: ISSL_GeometryCameraCalibration, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Encodes the specified SSL_GeometryCameraCalibration message, length delimited. Does not implicitly {@link SSL_GeometryCameraCalibration.verify|verify} messages.
     * @param message SSL_GeometryCameraCalibration message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encodeDelimited(message: ISSL_GeometryCameraCalibration, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a SSL_GeometryCameraCalibration message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns SSL_GeometryCameraCalibration
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): SSL_GeometryCameraCalibration;

    /**
     * Decodes a SSL_GeometryCameraCalibration message from the specified reader or buffer, length delimited.
     * @param reader Reader or buffer to decode from
     * @returns SSL_GeometryCameraCalibration
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): SSL_GeometryCameraCalibration;

    /**
     * Verifies a SSL_GeometryCameraCalibration message.
     * @param message Plain object to verify
     * @returns `null` if valid, otherwise the reason why it is not
     */
    public static verify(message: { [k: string]: any }): (string|null);

    /**
     * Creates a SSL_GeometryCameraCalibration message from a plain object. Also converts values to their respective internal types.
     * @param object Plain object
     * @returns SSL_GeometryCameraCalibration
     */
    public static fromObject(object: { [k: string]: any }): SSL_GeometryCameraCalibration;

    /**
     * Creates a plain object from a SSL_GeometryCameraCalibration message. Also converts values to other types if specified.
     * @param message SSL_GeometryCameraCalibration
     * @param [options] Conversion options
     * @returns Plain object
     */
    public static toObject(message: SSL_GeometryCameraCalibration, options?: $protobuf.IConversionOptions): { [k: string]: any };

    /**
     * Converts this SSL_GeometryCameraCalibration to JSON.
     * @returns JSON object
     */
    public toJSON(): { [k: string]: any };

    /**
     * Gets the default type url for SSL_GeometryCameraCalibration
     * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
     * @returns The default type url
     */
    public static getTypeUrl(typeUrlPrefix?: string): string;
}

/** Represents a SSL_BallModelStraightTwoPhase. */
export class SSL_BallModelStraightTwoPhase implements ISSL_BallModelStraightTwoPhase {

    /**
     * Constructs a new SSL_BallModelStraightTwoPhase.
     * @param [properties] Properties to set
     */
    constructor(properties?: ISSL_BallModelStraightTwoPhase);

    /** SSL_BallModelStraightTwoPhase accSlide. */
    public accSlide: number;

    /** SSL_BallModelStraightTwoPhase accRoll. */
    public accRoll: number;

    /** SSL_BallModelStraightTwoPhase kSwitch. */
    public kSwitch: number;

    /**
     * Creates a new SSL_BallModelStraightTwoPhase instance using the specified properties.
     * @param [properties] Properties to set
     * @returns SSL_BallModelStraightTwoPhase instance
     */
    public static create(properties?: ISSL_BallModelStraightTwoPhase): SSL_BallModelStraightTwoPhase;

    /**
     * Encodes the specified SSL_BallModelStraightTwoPhase message. Does not implicitly {@link SSL_BallModelStraightTwoPhase.verify|verify} messages.
     * @param message SSL_BallModelStraightTwoPhase message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: ISSL_BallModelStraightTwoPhase, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Encodes the specified SSL_BallModelStraightTwoPhase message, length delimited. Does not implicitly {@link SSL_BallModelStraightTwoPhase.verify|verify} messages.
     * @param message SSL_BallModelStraightTwoPhase message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encodeDelimited(message: ISSL_BallModelStraightTwoPhase, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a SSL_BallModelStraightTwoPhase message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns SSL_BallModelStraightTwoPhase
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): SSL_BallModelStraightTwoPhase;

    /**
     * Decodes a SSL_BallModelStraightTwoPhase message from the specified reader or buffer, length delimited.
     * @param reader Reader or buffer to decode from
     * @returns SSL_BallModelStraightTwoPhase
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): SSL_BallModelStraightTwoPhase;

    /**
     * Verifies a SSL_BallModelStraightTwoPhase message.
     * @param message Plain object to verify
     * @returns `null` if valid, otherwise the reason why it is not
     */
    public static verify(message: { [k: string]: any }): (string|null);

    /**
     * Creates a SSL_BallModelStraightTwoPhase message from a plain object. Also converts values to their respective internal types.
     * @param object Plain object
     * @returns SSL_BallModelStraightTwoPhase
     */
    public static fromObject(object: { [k: string]: any }): SSL_BallModelStraightTwoPhase;

    /**
     * Creates a plain object from a SSL_BallModelStraightTwoPhase message. Also converts values to other types if specified.
     * @param message SSL_BallModelStraightTwoPhase
     * @param [options] Conversion options
     * @returns Plain object
     */
    public static toObject(message: SSL_BallModelStraightTwoPhase, options?: $protobuf.IConversionOptions): { [k: string]: any };

    /**
     * Converts this SSL_BallModelStraightTwoPhase to JSON.
     * @returns JSON object
     */
    public toJSON(): { [k: string]: any };

    /**
     * Gets the default type url for SSL_BallModelStraightTwoPhase
     * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
     * @returns The default type url
     */
    public static getTypeUrl(typeUrlPrefix?: string): string;
}

/** Represents a SSL_BallModelChipFixedLoss. */
export class SSL_BallModelChipFixedLoss implements ISSL_BallModelChipFixedLoss {

    /**
     * Constructs a new SSL_BallModelChipFixedLoss.
     * @param [properties] Properties to set
     */
    constructor(properties?: ISSL_BallModelChipFixedLoss);

    /** SSL_BallModelChipFixedLoss dampingXyFirstHop. */
    public dampingXyFirstHop: number;

    /** SSL_BallModelChipFixedLoss dampingXyOtherHops. */
    public dampingXyOtherHops: number;

    /** SSL_BallModelChipFixedLoss dampingZ. */
    public dampingZ: number;

    /**
     * Creates a new SSL_BallModelChipFixedLoss instance using the specified properties.
     * @param [properties] Properties to set
     * @returns SSL_BallModelChipFixedLoss instance
     */
    public static create(properties?: ISSL_BallModelChipFixedLoss): SSL_BallModelChipFixedLoss;

    /**
     * Encodes the specified SSL_BallModelChipFixedLoss message. Does not implicitly {@link SSL_BallModelChipFixedLoss.verify|verify} messages.
     * @param message SSL_BallModelChipFixedLoss message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: ISSL_BallModelChipFixedLoss, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Encodes the specified SSL_BallModelChipFixedLoss message, length delimited. Does not implicitly {@link SSL_BallModelChipFixedLoss.verify|verify} messages.
     * @param message SSL_BallModelChipFixedLoss message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encodeDelimited(message: ISSL_BallModelChipFixedLoss, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a SSL_BallModelChipFixedLoss message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns SSL_BallModelChipFixedLoss
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): SSL_BallModelChipFixedLoss;

    /**
     * Decodes a SSL_BallModelChipFixedLoss message from the specified reader or buffer, length delimited.
     * @param reader Reader or buffer to decode from
     * @returns SSL_BallModelChipFixedLoss
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): SSL_BallModelChipFixedLoss;

    /**
     * Verifies a SSL_BallModelChipFixedLoss message.
     * @param message Plain object to verify
     * @returns `null` if valid, otherwise the reason why it is not
     */
    public static verify(message: { [k: string]: any }): (string|null);

    /**
     * Creates a SSL_BallModelChipFixedLoss message from a plain object. Also converts values to their respective internal types.
     * @param object Plain object
     * @returns SSL_BallModelChipFixedLoss
     */
    public static fromObject(object: { [k: string]: any }): SSL_BallModelChipFixedLoss;

    /**
     * Creates a plain object from a SSL_BallModelChipFixedLoss message. Also converts values to other types if specified.
     * @param message SSL_BallModelChipFixedLoss
     * @param [options] Conversion options
     * @returns Plain object
     */
    public static toObject(message: SSL_BallModelChipFixedLoss, options?: $protobuf.IConversionOptions): { [k: string]: any };

    /**
     * Converts this SSL_BallModelChipFixedLoss to JSON.
     * @returns JSON object
     */
    public toJSON(): { [k: string]: any };

    /**
     * Gets the default type url for SSL_BallModelChipFixedLoss
     * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
     * @returns The default type url
     */
    public static getTypeUrl(typeUrlPrefix?: string): string;
}

/** Represents a SSL_GeometryModels. */
export class SSL_GeometryModels implements ISSL_GeometryModels {

    /**
     * Constructs a new SSL_GeometryModels.
     * @param [properties] Properties to set
     */
    constructor(properties?: ISSL_GeometryModels);

    /** SSL_GeometryModels straightTwoPhase. */
    public straightTwoPhase?: (ISSL_BallModelStraightTwoPhase|null);

    /** SSL_GeometryModels chipFixedLoss. */
    public chipFixedLoss?: (ISSL_BallModelChipFixedLoss|null);

    /**
     * Creates a new SSL_GeometryModels instance using the specified properties.
     * @param [properties] Properties to set
     * @returns SSL_GeometryModels instance
     */
    public static create(properties?: ISSL_GeometryModels): SSL_GeometryModels;

    /**
     * Encodes the specified SSL_GeometryModels message. Does not implicitly {@link SSL_GeometryModels.verify|verify} messages.
     * @param message SSL_GeometryModels message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: ISSL_GeometryModels, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Encodes the specified SSL_GeometryModels message, length delimited. Does not implicitly {@link SSL_GeometryModels.verify|verify} messages.
     * @param message SSL_GeometryModels message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encodeDelimited(message: ISSL_GeometryModels, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a SSL_GeometryModels message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns SSL_GeometryModels
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): SSL_GeometryModels;

    /**
     * Decodes a SSL_GeometryModels message from the specified reader or buffer, length delimited.
     * @param reader Reader or buffer to decode from
     * @returns SSL_GeometryModels
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): SSL_GeometryModels;

    /**
     * Verifies a SSL_GeometryModels message.
     * @param message Plain object to verify
     * @returns `null` if valid, otherwise the reason why it is not
     */
    public static verify(message: { [k: string]: any }): (string|null);

    /**
     * Creates a SSL_GeometryModels message from a plain object. Also converts values to their respective internal types.
     * @param object Plain object
     * @returns SSL_GeometryModels
     */
    public static fromObject(object: { [k: string]: any }): SSL_GeometryModels;

    /**
     * Creates a plain object from a SSL_GeometryModels message. Also converts values to other types if specified.
     * @param message SSL_GeometryModels
     * @param [options] Conversion options
     * @returns Plain object
     */
    public static toObject(message: SSL_GeometryModels, options?: $protobuf.IConversionOptions): { [k: string]: any };

    /**
     * Converts this SSL_GeometryModels to JSON.
     * @returns JSON object
     */
    public toJSON(): { [k: string]: any };

    /**
     * Gets the default type url for SSL_GeometryModels
     * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
     * @returns The default type url
     */
    public static getTypeUrl(typeUrlPrefix?: string): string;
}

/** Represents a SSL_GeometryData. */
export class SSL_GeometryData implements ISSL_GeometryData {

    /**
     * Constructs a new SSL_GeometryData.
     * @param [properties] Properties to set
     */
    constructor(properties?: ISSL_GeometryData);

    /** SSL_GeometryData field. */
    public field: ISSL_GeometryFieldSize;

    /** SSL_GeometryData calib. */
    public calib: ISSL_GeometryCameraCalibration[];

    /** SSL_GeometryData models. */
    public models?: (ISSL_GeometryModels|null);

    /**
     * Creates a new SSL_GeometryData instance using the specified properties.
     * @param [properties] Properties to set
     * @returns SSL_GeometryData instance
     */
    public static create(properties?: ISSL_GeometryData): SSL_GeometryData;

    /**
     * Encodes the specified SSL_GeometryData message. Does not implicitly {@link SSL_GeometryData.verify|verify} messages.
     * @param message SSL_GeometryData message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: ISSL_GeometryData, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Encodes the specified SSL_GeometryData message, length delimited. Does not implicitly {@link SSL_GeometryData.verify|verify} messages.
     * @param message SSL_GeometryData message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encodeDelimited(message: ISSL_GeometryData, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a SSL_GeometryData message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns SSL_GeometryData
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): SSL_GeometryData;

    /**
     * Decodes a SSL_GeometryData message from the specified reader or buffer, length delimited.
     * @param reader Reader or buffer to decode from
     * @returns SSL_GeometryData
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): SSL_GeometryData;

    /**
     * Verifies a SSL_GeometryData message.
     * @param message Plain object to verify
     * @returns `null` if valid, otherwise the reason why it is not
     */
    public static verify(message: { [k: string]: any }): (string|null);

    /**
     * Creates a SSL_GeometryData message from a plain object. Also converts values to their respective internal types.
     * @param object Plain object
     * @returns SSL_GeometryData
     */
    public static fromObject(object: { [k: string]: any }): SSL_GeometryData;

    /**
     * Creates a plain object from a SSL_GeometryData message. Also converts values to other types if specified.
     * @param message SSL_GeometryData
     * @param [options] Conversion options
     * @returns Plain object
     */
    public static toObject(message: SSL_GeometryData, options?: $protobuf.IConversionOptions): { [k: string]: any };

    /**
     * Converts this SSL_GeometryData to JSON.
     * @returns JSON object
     */
    public toJSON(): { [k: string]: any };

    /**
     * Gets the default type url for SSL_GeometryData
     * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
     * @returns The default type url
     */
    public static getTypeUrl(typeUrlPrefix?: string): string;
}

/** SSL_FieldShapeType enum. */
export enum SSL_FieldShapeType {
    Undefined = 0,
    CenterCircle = 1,
    TopTouchLine = 2,
    BottomTouchLine = 3,
    LeftGoalLine = 4,
    RightGoalLine = 5,
    HalfwayLine = 6,
    CenterLine = 7,
    LeftPenaltyStretch = 8,
    RightPenaltyStretch = 9,
    LeftFieldLeftPenaltyStretch = 10,
    LeftFieldRightPenaltyStretch = 11,
    RightFieldLeftPenaltyStretch = 12,
    RightFieldRightPenaltyStretch = 13
}

/** Namespace google. */
export namespace google {

    /** Namespace protobuf. */
    namespace protobuf {

        /** Properties of an Any. */
        interface IAny {

            /** Any type_url */
            type_url?: (string|null);

            /** Any value */
            value?: (Uint8Array|null);
        }

        /** Represents an Any. */
        class Any implements IAny {

            /**
             * Constructs a new Any.
             * @param [properties] Properties to set
             */
            constructor(properties?: google.protobuf.IAny);

            /** Any type_url. */
            public type_url: string;

            /** Any value. */
            public value: Uint8Array;

            /**
             * Creates a new Any instance using the specified properties.
             * @param [properties] Properties to set
             * @returns Any instance
             */
            public static create(properties?: google.protobuf.IAny): google.protobuf.Any;

            /**
             * Encodes the specified Any message. Does not implicitly {@link google.protobuf.Any.verify|verify} messages.
             * @param message Any message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encode(message: google.protobuf.IAny, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Encodes the specified Any message, length delimited. Does not implicitly {@link google.protobuf.Any.verify|verify} messages.
             * @param message Any message or plain object to encode
             * @param [writer] Writer to encode to
             * @returns Writer
             */
            public static encodeDelimited(message: google.protobuf.IAny, writer?: $protobuf.Writer): $protobuf.Writer;

            /**
             * Decodes an Any message from the specified reader or buffer.
             * @param reader Reader or buffer to decode from
             * @param [length] Message length if known beforehand
             * @returns Any
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): google.protobuf.Any;

            /**
             * Decodes an Any message from the specified reader or buffer, length delimited.
             * @param reader Reader or buffer to decode from
             * @returns Any
             * @throws {Error} If the payload is not a reader or valid buffer
             * @throws {$protobuf.util.ProtocolError} If required fields are missing
             */
            public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): google.protobuf.Any;

            /**
             * Verifies an Any message.
             * @param message Plain object to verify
             * @returns `null` if valid, otherwise the reason why it is not
             */
            public static verify(message: { [k: string]: any }): (string|null);

            /**
             * Creates an Any message from a plain object. Also converts values to their respective internal types.
             * @param object Plain object
             * @returns Any
             */
            public static fromObject(object: { [k: string]: any }): google.protobuf.Any;

            /**
             * Creates a plain object from an Any message. Also converts values to other types if specified.
             * @param message Any
             * @param [options] Conversion options
             * @returns Plain object
             */
            public static toObject(message: google.protobuf.Any, options?: $protobuf.IConversionOptions): { [k: string]: any };

            /**
             * Converts this Any to JSON.
             * @returns JSON object
             */
            public toJSON(): { [k: string]: any };

            /**
             * Gets the default type url for Any
             * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
             * @returns The default type url
             */
            public static getTypeUrl(typeUrlPrefix?: string): string;
        }
    }
}

/** Represents a SimulatorError. */
export class SimulatorError implements ISimulatorError {

    /**
     * Constructs a new SimulatorError.
     * @param [properties] Properties to set
     */
    constructor(properties?: ISimulatorError);

    /** SimulatorError code. */
    public code: string;

    /** SimulatorError message. */
    public message: string;

    /**
     * Creates a new SimulatorError instance using the specified properties.
     * @param [properties] Properties to set
     * @returns SimulatorError instance
     */
    public static create(properties?: ISimulatorError): SimulatorError;

    /**
     * Encodes the specified SimulatorError message. Does not implicitly {@link SimulatorError.verify|verify} messages.
     * @param message SimulatorError message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: ISimulatorError, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Encodes the specified SimulatorError message, length delimited. Does not implicitly {@link SimulatorError.verify|verify} messages.
     * @param message SimulatorError message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encodeDelimited(message: ISimulatorError, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a SimulatorError message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns SimulatorError
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): SimulatorError;

    /**
     * Decodes a SimulatorError message from the specified reader or buffer, length delimited.
     * @param reader Reader or buffer to decode from
     * @returns SimulatorError
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): SimulatorError;

    /**
     * Verifies a SimulatorError message.
     * @param message Plain object to verify
     * @returns `null` if valid, otherwise the reason why it is not
     */
    public static verify(message: { [k: string]: any }): (string|null);

    /**
     * Creates a SimulatorError message from a plain object. Also converts values to their respective internal types.
     * @param object Plain object
     * @returns SimulatorError
     */
    public static fromObject(object: { [k: string]: any }): SimulatorError;

    /**
     * Creates a plain object from a SimulatorError message. Also converts values to other types if specified.
     * @param message SimulatorError
     * @param [options] Conversion options
     * @returns Plain object
     */
    public static toObject(message: SimulatorError, options?: $protobuf.IConversionOptions): { [k: string]: any };

    /**
     * Converts this SimulatorError to JSON.
     * @returns JSON object
     */
    public toJSON(): { [k: string]: any };

    /**
     * Gets the default type url for SimulatorError
     * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
     * @returns The default type url
     */
    public static getTypeUrl(typeUrlPrefix?: string): string;
}

/** Represents a RobotCommand. */
export class RobotCommand implements IRobotCommand {

    /**
     * Constructs a new RobotCommand.
     * @param [properties] Properties to set
     */
    constructor(properties?: IRobotCommand);

    /** RobotCommand id. */
    public id: number;

    /** RobotCommand moveCommand. */
    public moveCommand?: (IRobotMoveCommand|null);

    /** RobotCommand kickSpeed. */
    public kickSpeed: number;

    /** RobotCommand kickAngle. */
    public kickAngle: number;

    /** RobotCommand dribblerSpeed. */
    public dribblerSpeed: number;

    /**
     * Creates a new RobotCommand instance using the specified properties.
     * @param [properties] Properties to set
     * @returns RobotCommand instance
     */
    public static create(properties?: IRobotCommand): RobotCommand;

    /**
     * Encodes the specified RobotCommand message. Does not implicitly {@link RobotCommand.verify|verify} messages.
     * @param message RobotCommand message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: IRobotCommand, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Encodes the specified RobotCommand message, length delimited. Does not implicitly {@link RobotCommand.verify|verify} messages.
     * @param message RobotCommand message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encodeDelimited(message: IRobotCommand, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a RobotCommand message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns RobotCommand
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): RobotCommand;

    /**
     * Decodes a RobotCommand message from the specified reader or buffer, length delimited.
     * @param reader Reader or buffer to decode from
     * @returns RobotCommand
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): RobotCommand;

    /**
     * Verifies a RobotCommand message.
     * @param message Plain object to verify
     * @returns `null` if valid, otherwise the reason why it is not
     */
    public static verify(message: { [k: string]: any }): (string|null);

    /**
     * Creates a RobotCommand message from a plain object. Also converts values to their respective internal types.
     * @param object Plain object
     * @returns RobotCommand
     */
    public static fromObject(object: { [k: string]: any }): RobotCommand;

    /**
     * Creates a plain object from a RobotCommand message. Also converts values to other types if specified.
     * @param message RobotCommand
     * @param [options] Conversion options
     * @returns Plain object
     */
    public static toObject(message: RobotCommand, options?: $protobuf.IConversionOptions): { [k: string]: any };

    /**
     * Converts this RobotCommand to JSON.
     * @returns JSON object
     */
    public toJSON(): { [k: string]: any };

    /**
     * Gets the default type url for RobotCommand
     * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
     * @returns The default type url
     */
    public static getTypeUrl(typeUrlPrefix?: string): string;
}

/** Represents a RobotMoveCommand. */
export class RobotMoveCommand implements IRobotMoveCommand {

    /**
     * Constructs a new RobotMoveCommand.
     * @param [properties] Properties to set
     */
    constructor(properties?: IRobotMoveCommand);

    /** RobotMoveCommand wheelVelocity. */
    public wheelVelocity?: (IMoveWheelVelocity|null);

    /** RobotMoveCommand localVelocity. */
    public localVelocity?: (IMoveLocalVelocity|null);

    /** RobotMoveCommand globalVelocity. */
    public globalVelocity?: (IMoveGlobalVelocity|null);

    /** RobotMoveCommand command. */
    public command?: ("wheelVelocity"|"localVelocity"|"globalVelocity");

    /**
     * Creates a new RobotMoveCommand instance using the specified properties.
     * @param [properties] Properties to set
     * @returns RobotMoveCommand instance
     */
    public static create(properties?: IRobotMoveCommand): RobotMoveCommand;

    /**
     * Encodes the specified RobotMoveCommand message. Does not implicitly {@link RobotMoveCommand.verify|verify} messages.
     * @param message RobotMoveCommand message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: IRobotMoveCommand, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Encodes the specified RobotMoveCommand message, length delimited. Does not implicitly {@link RobotMoveCommand.verify|verify} messages.
     * @param message RobotMoveCommand message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encodeDelimited(message: IRobotMoveCommand, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a RobotMoveCommand message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns RobotMoveCommand
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): RobotMoveCommand;

    /**
     * Decodes a RobotMoveCommand message from the specified reader or buffer, length delimited.
     * @param reader Reader or buffer to decode from
     * @returns RobotMoveCommand
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): RobotMoveCommand;

    /**
     * Verifies a RobotMoveCommand message.
     * @param message Plain object to verify
     * @returns `null` if valid, otherwise the reason why it is not
     */
    public static verify(message: { [k: string]: any }): (string|null);

    /**
     * Creates a RobotMoveCommand message from a plain object. Also converts values to their respective internal types.
     * @param object Plain object
     * @returns RobotMoveCommand
     */
    public static fromObject(object: { [k: string]: any }): RobotMoveCommand;

    /**
     * Creates a plain object from a RobotMoveCommand message. Also converts values to other types if specified.
     * @param message RobotMoveCommand
     * @param [options] Conversion options
     * @returns Plain object
     */
    public static toObject(message: RobotMoveCommand, options?: $protobuf.IConversionOptions): { [k: string]: any };

    /**
     * Converts this RobotMoveCommand to JSON.
     * @returns JSON object
     */
    public toJSON(): { [k: string]: any };

    /**
     * Gets the default type url for RobotMoveCommand
     * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
     * @returns The default type url
     */
    public static getTypeUrl(typeUrlPrefix?: string): string;
}

/** Represents a MoveWheelVelocity. */
export class MoveWheelVelocity implements IMoveWheelVelocity {

    /**
     * Constructs a new MoveWheelVelocity.
     * @param [properties] Properties to set
     */
    constructor(properties?: IMoveWheelVelocity);

    /** MoveWheelVelocity frontRight. */
    public frontRight: number;

    /** MoveWheelVelocity backRight. */
    public backRight: number;

    /** MoveWheelVelocity backLeft. */
    public backLeft: number;

    /** MoveWheelVelocity frontLeft. */
    public frontLeft: number;

    /**
     * Creates a new MoveWheelVelocity instance using the specified properties.
     * @param [properties] Properties to set
     * @returns MoveWheelVelocity instance
     */
    public static create(properties?: IMoveWheelVelocity): MoveWheelVelocity;

    /**
     * Encodes the specified MoveWheelVelocity message. Does not implicitly {@link MoveWheelVelocity.verify|verify} messages.
     * @param message MoveWheelVelocity message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: IMoveWheelVelocity, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Encodes the specified MoveWheelVelocity message, length delimited. Does not implicitly {@link MoveWheelVelocity.verify|verify} messages.
     * @param message MoveWheelVelocity message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encodeDelimited(message: IMoveWheelVelocity, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a MoveWheelVelocity message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns MoveWheelVelocity
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): MoveWheelVelocity;

    /**
     * Decodes a MoveWheelVelocity message from the specified reader or buffer, length delimited.
     * @param reader Reader or buffer to decode from
     * @returns MoveWheelVelocity
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): MoveWheelVelocity;

    /**
     * Verifies a MoveWheelVelocity message.
     * @param message Plain object to verify
     * @returns `null` if valid, otherwise the reason why it is not
     */
    public static verify(message: { [k: string]: any }): (string|null);

    /**
     * Creates a MoveWheelVelocity message from a plain object. Also converts values to their respective internal types.
     * @param object Plain object
     * @returns MoveWheelVelocity
     */
    public static fromObject(object: { [k: string]: any }): MoveWheelVelocity;

    /**
     * Creates a plain object from a MoveWheelVelocity message. Also converts values to other types if specified.
     * @param message MoveWheelVelocity
     * @param [options] Conversion options
     * @returns Plain object
     */
    public static toObject(message: MoveWheelVelocity, options?: $protobuf.IConversionOptions): { [k: string]: any };

    /**
     * Converts this MoveWheelVelocity to JSON.
     * @returns JSON object
     */
    public toJSON(): { [k: string]: any };

    /**
     * Gets the default type url for MoveWheelVelocity
     * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
     * @returns The default type url
     */
    public static getTypeUrl(typeUrlPrefix?: string): string;
}

/** Represents a MoveLocalVelocity. */
export class MoveLocalVelocity implements IMoveLocalVelocity {

    /**
     * Constructs a new MoveLocalVelocity.
     * @param [properties] Properties to set
     */
    constructor(properties?: IMoveLocalVelocity);

    /** MoveLocalVelocity forward. */
    public forward: number;

    /** MoveLocalVelocity left. */
    public left: number;

    /** MoveLocalVelocity angular. */
    public angular: number;

    /**
     * Creates a new MoveLocalVelocity instance using the specified properties.
     * @param [properties] Properties to set
     * @returns MoveLocalVelocity instance
     */
    public static create(properties?: IMoveLocalVelocity): MoveLocalVelocity;

    /**
     * Encodes the specified MoveLocalVelocity message. Does not implicitly {@link MoveLocalVelocity.verify|verify} messages.
     * @param message MoveLocalVelocity message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: IMoveLocalVelocity, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Encodes the specified MoveLocalVelocity message, length delimited. Does not implicitly {@link MoveLocalVelocity.verify|verify} messages.
     * @param message MoveLocalVelocity message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encodeDelimited(message: IMoveLocalVelocity, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a MoveLocalVelocity message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns MoveLocalVelocity
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): MoveLocalVelocity;

    /**
     * Decodes a MoveLocalVelocity message from the specified reader or buffer, length delimited.
     * @param reader Reader or buffer to decode from
     * @returns MoveLocalVelocity
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): MoveLocalVelocity;

    /**
     * Verifies a MoveLocalVelocity message.
     * @param message Plain object to verify
     * @returns `null` if valid, otherwise the reason why it is not
     */
    public static verify(message: { [k: string]: any }): (string|null);

    /**
     * Creates a MoveLocalVelocity message from a plain object. Also converts values to their respective internal types.
     * @param object Plain object
     * @returns MoveLocalVelocity
     */
    public static fromObject(object: { [k: string]: any }): MoveLocalVelocity;

    /**
     * Creates a plain object from a MoveLocalVelocity message. Also converts values to other types if specified.
     * @param message MoveLocalVelocity
     * @param [options] Conversion options
     * @returns Plain object
     */
    public static toObject(message: MoveLocalVelocity, options?: $protobuf.IConversionOptions): { [k: string]: any };

    /**
     * Converts this MoveLocalVelocity to JSON.
     * @returns JSON object
     */
    public toJSON(): { [k: string]: any };

    /**
     * Gets the default type url for MoveLocalVelocity
     * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
     * @returns The default type url
     */
    public static getTypeUrl(typeUrlPrefix?: string): string;
}

/** Represents a MoveGlobalVelocity. */
export class MoveGlobalVelocity implements IMoveGlobalVelocity {

    /**
     * Constructs a new MoveGlobalVelocity.
     * @param [properties] Properties to set
     */
    constructor(properties?: IMoveGlobalVelocity);

    /** MoveGlobalVelocity x. */
    public x: number;

    /** MoveGlobalVelocity y. */
    public y: number;

    /** MoveGlobalVelocity angular. */
    public angular: number;

    /**
     * Creates a new MoveGlobalVelocity instance using the specified properties.
     * @param [properties] Properties to set
     * @returns MoveGlobalVelocity instance
     */
    public static create(properties?: IMoveGlobalVelocity): MoveGlobalVelocity;

    /**
     * Encodes the specified MoveGlobalVelocity message. Does not implicitly {@link MoveGlobalVelocity.verify|verify} messages.
     * @param message MoveGlobalVelocity message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: IMoveGlobalVelocity, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Encodes the specified MoveGlobalVelocity message, length delimited. Does not implicitly {@link MoveGlobalVelocity.verify|verify} messages.
     * @param message MoveGlobalVelocity message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encodeDelimited(message: IMoveGlobalVelocity, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a MoveGlobalVelocity message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns MoveGlobalVelocity
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): MoveGlobalVelocity;

    /**
     * Decodes a MoveGlobalVelocity message from the specified reader or buffer, length delimited.
     * @param reader Reader or buffer to decode from
     * @returns MoveGlobalVelocity
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): MoveGlobalVelocity;

    /**
     * Verifies a MoveGlobalVelocity message.
     * @param message Plain object to verify
     * @returns `null` if valid, otherwise the reason why it is not
     */
    public static verify(message: { [k: string]: any }): (string|null);

    /**
     * Creates a MoveGlobalVelocity message from a plain object. Also converts values to their respective internal types.
     * @param object Plain object
     * @returns MoveGlobalVelocity
     */
    public static fromObject(object: { [k: string]: any }): MoveGlobalVelocity;

    /**
     * Creates a plain object from a MoveGlobalVelocity message. Also converts values to other types if specified.
     * @param message MoveGlobalVelocity
     * @param [options] Conversion options
     * @returns Plain object
     */
    public static toObject(message: MoveGlobalVelocity, options?: $protobuf.IConversionOptions): { [k: string]: any };

    /**
     * Converts this MoveGlobalVelocity to JSON.
     * @returns JSON object
     */
    public toJSON(): { [k: string]: any };

    /**
     * Gets the default type url for MoveGlobalVelocity
     * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
     * @returns The default type url
     */
    public static getTypeUrl(typeUrlPrefix?: string): string;
}

/** Represents a RobotControl. */
export class RobotControl implements IRobotControl {

    /**
     * Constructs a new RobotControl.
     * @param [properties] Properties to set
     */
    constructor(properties?: IRobotControl);

    /** RobotControl robotCommands. */
    public robotCommands: IRobotCommand[];

    /**
     * Creates a new RobotControl instance using the specified properties.
     * @param [properties] Properties to set
     * @returns RobotControl instance
     */
    public static create(properties?: IRobotControl): RobotControl;

    /**
     * Encodes the specified RobotControl message. Does not implicitly {@link RobotControl.verify|verify} messages.
     * @param message RobotControl message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: IRobotControl, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Encodes the specified RobotControl message, length delimited. Does not implicitly {@link RobotControl.verify|verify} messages.
     * @param message RobotControl message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encodeDelimited(message: IRobotControl, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a RobotControl message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns RobotControl
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): RobotControl;

    /**
     * Decodes a RobotControl message from the specified reader or buffer, length delimited.
     * @param reader Reader or buffer to decode from
     * @returns RobotControl
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): RobotControl;

    /**
     * Verifies a RobotControl message.
     * @param message Plain object to verify
     * @returns `null` if valid, otherwise the reason why it is not
     */
    public static verify(message: { [k: string]: any }): (string|null);

    /**
     * Creates a RobotControl message from a plain object. Also converts values to their respective internal types.
     * @param object Plain object
     * @returns RobotControl
     */
    public static fromObject(object: { [k: string]: any }): RobotControl;

    /**
     * Creates a plain object from a RobotControl message. Also converts values to other types if specified.
     * @param message RobotControl
     * @param [options] Conversion options
     * @returns Plain object
     */
    public static toObject(message: RobotControl, options?: $protobuf.IConversionOptions): { [k: string]: any };

    /**
     * Converts this RobotControl to JSON.
     * @returns JSON object
     */
    public toJSON(): { [k: string]: any };

    /**
     * Gets the default type url for RobotControl
     * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
     * @returns The default type url
     */
    public static getTypeUrl(typeUrlPrefix?: string): string;
}

/** Represents a RobotFeedback. */
export class RobotFeedback implements IRobotFeedback {

    /**
     * Constructs a new RobotFeedback.
     * @param [properties] Properties to set
     */
    constructor(properties?: IRobotFeedback);

    /** RobotFeedback id. */
    public id: number;

    /** RobotFeedback dribblerBallContact. */
    public dribblerBallContact: boolean;

    /** RobotFeedback custom. */
    public custom?: (google.protobuf.IAny|null);

    /**
     * Creates a new RobotFeedback instance using the specified properties.
     * @param [properties] Properties to set
     * @returns RobotFeedback instance
     */
    public static create(properties?: IRobotFeedback): RobotFeedback;

    /**
     * Encodes the specified RobotFeedback message. Does not implicitly {@link RobotFeedback.verify|verify} messages.
     * @param message RobotFeedback message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: IRobotFeedback, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Encodes the specified RobotFeedback message, length delimited. Does not implicitly {@link RobotFeedback.verify|verify} messages.
     * @param message RobotFeedback message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encodeDelimited(message: IRobotFeedback, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a RobotFeedback message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns RobotFeedback
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): RobotFeedback;

    /**
     * Decodes a RobotFeedback message from the specified reader or buffer, length delimited.
     * @param reader Reader or buffer to decode from
     * @returns RobotFeedback
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): RobotFeedback;

    /**
     * Verifies a RobotFeedback message.
     * @param message Plain object to verify
     * @returns `null` if valid, otherwise the reason why it is not
     */
    public static verify(message: { [k: string]: any }): (string|null);

    /**
     * Creates a RobotFeedback message from a plain object. Also converts values to their respective internal types.
     * @param object Plain object
     * @returns RobotFeedback
     */
    public static fromObject(object: { [k: string]: any }): RobotFeedback;

    /**
     * Creates a plain object from a RobotFeedback message. Also converts values to other types if specified.
     * @param message RobotFeedback
     * @param [options] Conversion options
     * @returns Plain object
     */
    public static toObject(message: RobotFeedback, options?: $protobuf.IConversionOptions): { [k: string]: any };

    /**
     * Converts this RobotFeedback to JSON.
     * @returns JSON object
     */
    public toJSON(): { [k: string]: any };

    /**
     * Gets the default type url for RobotFeedback
     * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
     * @returns The default type url
     */
    public static getTypeUrl(typeUrlPrefix?: string): string;
}

/** Represents a RobotControlResponse. */
export class RobotControlResponse implements IRobotControlResponse {

    /**
     * Constructs a new RobotControlResponse.
     * @param [properties] Properties to set
     */
    constructor(properties?: IRobotControlResponse);

    /** RobotControlResponse errors. */
    public errors: ISimulatorError[];

    /** RobotControlResponse feedback. */
    public feedback: IRobotFeedback[];

    /**
     * Creates a new RobotControlResponse instance using the specified properties.
     * @param [properties] Properties to set
     * @returns RobotControlResponse instance
     */
    public static create(properties?: IRobotControlResponse): RobotControlResponse;

    /**
     * Encodes the specified RobotControlResponse message. Does not implicitly {@link RobotControlResponse.verify|verify} messages.
     * @param message RobotControlResponse message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: IRobotControlResponse, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Encodes the specified RobotControlResponse message, length delimited. Does not implicitly {@link RobotControlResponse.verify|verify} messages.
     * @param message RobotControlResponse message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encodeDelimited(message: IRobotControlResponse, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a RobotControlResponse message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns RobotControlResponse
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): RobotControlResponse;

    /**
     * Decodes a RobotControlResponse message from the specified reader or buffer, length delimited.
     * @param reader Reader or buffer to decode from
     * @returns RobotControlResponse
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): RobotControlResponse;

    /**
     * Verifies a RobotControlResponse message.
     * @param message Plain object to verify
     * @returns `null` if valid, otherwise the reason why it is not
     */
    public static verify(message: { [k: string]: any }): (string|null);

    /**
     * Creates a RobotControlResponse message from a plain object. Also converts values to their respective internal types.
     * @param object Plain object
     * @returns RobotControlResponse
     */
    public static fromObject(object: { [k: string]: any }): RobotControlResponse;

    /**
     * Creates a plain object from a RobotControlResponse message. Also converts values to other types if specified.
     * @param message RobotControlResponse
     * @param [options] Conversion options
     * @returns Plain object
     */
    public static toObject(message: RobotControlResponse, options?: $protobuf.IConversionOptions): { [k: string]: any };

    /**
     * Converts this RobotControlResponse to JSON.
     * @returns JSON object
     */
    public toJSON(): { [k: string]: any };

    /**
     * Gets the default type url for RobotControlResponse
     * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
     * @returns The default type url
     */
    public static getTypeUrl(typeUrlPrefix?: string): string;
}

/** Represents a SimulationSyncRequest. */
export class SimulationSyncRequest implements ISimulationSyncRequest {

    /**
     * Constructs a new SimulationSyncRequest.
     * @param [properties] Properties to set
     */
    constructor(properties?: ISimulationSyncRequest);

    /** SimulationSyncRequest simStep. */
    public simStep: number;

    /** SimulationSyncRequest simulatorCommand. */
    public simulatorCommand?: (ISimulatorCommand|null);

    /** SimulationSyncRequest robotControl. */
    public robotControl?: (IRobotControl|null);

    /**
     * Creates a new SimulationSyncRequest instance using the specified properties.
     * @param [properties] Properties to set
     * @returns SimulationSyncRequest instance
     */
    public static create(properties?: ISimulationSyncRequest): SimulationSyncRequest;

    /**
     * Encodes the specified SimulationSyncRequest message. Does not implicitly {@link SimulationSyncRequest.verify|verify} messages.
     * @param message SimulationSyncRequest message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: ISimulationSyncRequest, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Encodes the specified SimulationSyncRequest message, length delimited. Does not implicitly {@link SimulationSyncRequest.verify|verify} messages.
     * @param message SimulationSyncRequest message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encodeDelimited(message: ISimulationSyncRequest, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a SimulationSyncRequest message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns SimulationSyncRequest
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): SimulationSyncRequest;

    /**
     * Decodes a SimulationSyncRequest message from the specified reader or buffer, length delimited.
     * @param reader Reader or buffer to decode from
     * @returns SimulationSyncRequest
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): SimulationSyncRequest;

    /**
     * Verifies a SimulationSyncRequest message.
     * @param message Plain object to verify
     * @returns `null` if valid, otherwise the reason why it is not
     */
    public static verify(message: { [k: string]: any }): (string|null);

    /**
     * Creates a SimulationSyncRequest message from a plain object. Also converts values to their respective internal types.
     * @param object Plain object
     * @returns SimulationSyncRequest
     */
    public static fromObject(object: { [k: string]: any }): SimulationSyncRequest;

    /**
     * Creates a plain object from a SimulationSyncRequest message. Also converts values to other types if specified.
     * @param message SimulationSyncRequest
     * @param [options] Conversion options
     * @returns Plain object
     */
    public static toObject(message: SimulationSyncRequest, options?: $protobuf.IConversionOptions): { [k: string]: any };

    /**
     * Converts this SimulationSyncRequest to JSON.
     * @returns JSON object
     */
    public toJSON(): { [k: string]: any };

    /**
     * Gets the default type url for SimulationSyncRequest
     * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
     * @returns The default type url
     */
    public static getTypeUrl(typeUrlPrefix?: string): string;
}

/** Represents a SimulationSyncResponse. */
export class SimulationSyncResponse implements ISimulationSyncResponse {

    /**
     * Constructs a new SimulationSyncResponse.
     * @param [properties] Properties to set
     */
    constructor(properties?: ISimulationSyncResponse);

    /** SimulationSyncResponse detection. */
    public detection: ISSL_DetectionFrame[];

    /** SimulationSyncResponse robotControlResponse. */
    public robotControlResponse?: (IRobotControlResponse|null);

    /**
     * Creates a new SimulationSyncResponse instance using the specified properties.
     * @param [properties] Properties to set
     * @returns SimulationSyncResponse instance
     */
    public static create(properties?: ISimulationSyncResponse): SimulationSyncResponse;

    /**
     * Encodes the specified SimulationSyncResponse message. Does not implicitly {@link SimulationSyncResponse.verify|verify} messages.
     * @param message SimulationSyncResponse message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: ISimulationSyncResponse, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Encodes the specified SimulationSyncResponse message, length delimited. Does not implicitly {@link SimulationSyncResponse.verify|verify} messages.
     * @param message SimulationSyncResponse message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encodeDelimited(message: ISimulationSyncResponse, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a SimulationSyncResponse message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns SimulationSyncResponse
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): SimulationSyncResponse;

    /**
     * Decodes a SimulationSyncResponse message from the specified reader or buffer, length delimited.
     * @param reader Reader or buffer to decode from
     * @returns SimulationSyncResponse
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): SimulationSyncResponse;

    /**
     * Verifies a SimulationSyncResponse message.
     * @param message Plain object to verify
     * @returns `null` if valid, otherwise the reason why it is not
     */
    public static verify(message: { [k: string]: any }): (string|null);

    /**
     * Creates a SimulationSyncResponse message from a plain object. Also converts values to their respective internal types.
     * @param object Plain object
     * @returns SimulationSyncResponse
     */
    public static fromObject(object: { [k: string]: any }): SimulationSyncResponse;

    /**
     * Creates a plain object from a SimulationSyncResponse message. Also converts values to other types if specified.
     * @param message SimulationSyncResponse
     * @param [options] Conversion options
     * @returns Plain object
     */
    public static toObject(message: SimulationSyncResponse, options?: $protobuf.IConversionOptions): { [k: string]: any };

    /**
     * Converts this SimulationSyncResponse to JSON.
     * @returns JSON object
     */
    public toJSON(): { [k: string]: any };

    /**
     * Gets the default type url for SimulationSyncResponse
     * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
     * @returns The default type url
     */
    public static getTypeUrl(typeUrlPrefix?: string): string;
}

/** Represents a SSL_DetectionBall. */
export class SSL_DetectionBall implements ISSL_DetectionBall {

    /**
     * Constructs a new SSL_DetectionBall.
     * @param [properties] Properties to set
     */
    constructor(properties?: ISSL_DetectionBall);

    /** SSL_DetectionBall confidence. */
    public confidence: number;

    /** SSL_DetectionBall area. */
    public area: number;

    /** SSL_DetectionBall x. */
    public x: number;

    /** SSL_DetectionBall y. */
    public y: number;

    /** SSL_DetectionBall z. */
    public z: number;

    /** SSL_DetectionBall pixelX. */
    public pixelX: number;

    /** SSL_DetectionBall pixelY. */
    public pixelY: number;

    /**
     * Creates a new SSL_DetectionBall instance using the specified properties.
     * @param [properties] Properties to set
     * @returns SSL_DetectionBall instance
     */
    public static create(properties?: ISSL_DetectionBall): SSL_DetectionBall;

    /**
     * Encodes the specified SSL_DetectionBall message. Does not implicitly {@link SSL_DetectionBall.verify|verify} messages.
     * @param message SSL_DetectionBall message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: ISSL_DetectionBall, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Encodes the specified SSL_DetectionBall message, length delimited. Does not implicitly {@link SSL_DetectionBall.verify|verify} messages.
     * @param message SSL_DetectionBall message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encodeDelimited(message: ISSL_DetectionBall, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a SSL_DetectionBall message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns SSL_DetectionBall
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): SSL_DetectionBall;

    /**
     * Decodes a SSL_DetectionBall message from the specified reader or buffer, length delimited.
     * @param reader Reader or buffer to decode from
     * @returns SSL_DetectionBall
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): SSL_DetectionBall;

    /**
     * Verifies a SSL_DetectionBall message.
     * @param message Plain object to verify
     * @returns `null` if valid, otherwise the reason why it is not
     */
    public static verify(message: { [k: string]: any }): (string|null);

    /**
     * Creates a SSL_DetectionBall message from a plain object. Also converts values to their respective internal types.
     * @param object Plain object
     * @returns SSL_DetectionBall
     */
    public static fromObject(object: { [k: string]: any }): SSL_DetectionBall;

    /**
     * Creates a plain object from a SSL_DetectionBall message. Also converts values to other types if specified.
     * @param message SSL_DetectionBall
     * @param [options] Conversion options
     * @returns Plain object
     */
    public static toObject(message: SSL_DetectionBall, options?: $protobuf.IConversionOptions): { [k: string]: any };

    /**
     * Converts this SSL_DetectionBall to JSON.
     * @returns JSON object
     */
    public toJSON(): { [k: string]: any };

    /**
     * Gets the default type url for SSL_DetectionBall
     * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
     * @returns The default type url
     */
    public static getTypeUrl(typeUrlPrefix?: string): string;
}

/** Represents a SSL_DetectionRobot. */
export class SSL_DetectionRobot implements ISSL_DetectionRobot {

    /**
     * Constructs a new SSL_DetectionRobot.
     * @param [properties] Properties to set
     */
    constructor(properties?: ISSL_DetectionRobot);

    /** SSL_DetectionRobot confidence. */
    public confidence: number;

    /** SSL_DetectionRobot robotId. */
    public robotId: number;

    /** SSL_DetectionRobot x. */
    public x: number;

    /** SSL_DetectionRobot y. */
    public y: number;

    /** SSL_DetectionRobot orientation. */
    public orientation: number;

    /** SSL_DetectionRobot pixelX. */
    public pixelX: number;

    /** SSL_DetectionRobot pixelY. */
    public pixelY: number;

    /** SSL_DetectionRobot height. */
    public height: number;

    /**
     * Creates a new SSL_DetectionRobot instance using the specified properties.
     * @param [properties] Properties to set
     * @returns SSL_DetectionRobot instance
     */
    public static create(properties?: ISSL_DetectionRobot): SSL_DetectionRobot;

    /**
     * Encodes the specified SSL_DetectionRobot message. Does not implicitly {@link SSL_DetectionRobot.verify|verify} messages.
     * @param message SSL_DetectionRobot message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: ISSL_DetectionRobot, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Encodes the specified SSL_DetectionRobot message, length delimited. Does not implicitly {@link SSL_DetectionRobot.verify|verify} messages.
     * @param message SSL_DetectionRobot message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encodeDelimited(message: ISSL_DetectionRobot, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a SSL_DetectionRobot message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns SSL_DetectionRobot
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): SSL_DetectionRobot;

    /**
     * Decodes a SSL_DetectionRobot message from the specified reader or buffer, length delimited.
     * @param reader Reader or buffer to decode from
     * @returns SSL_DetectionRobot
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): SSL_DetectionRobot;

    /**
     * Verifies a SSL_DetectionRobot message.
     * @param message Plain object to verify
     * @returns `null` if valid, otherwise the reason why it is not
     */
    public static verify(message: { [k: string]: any }): (string|null);

    /**
     * Creates a SSL_DetectionRobot message from a plain object. Also converts values to their respective internal types.
     * @param object Plain object
     * @returns SSL_DetectionRobot
     */
    public static fromObject(object: { [k: string]: any }): SSL_DetectionRobot;

    /**
     * Creates a plain object from a SSL_DetectionRobot message. Also converts values to other types if specified.
     * @param message SSL_DetectionRobot
     * @param [options] Conversion options
     * @returns Plain object
     */
    public static toObject(message: SSL_DetectionRobot, options?: $protobuf.IConversionOptions): { [k: string]: any };

    /**
     * Converts this SSL_DetectionRobot to JSON.
     * @returns JSON object
     */
    public toJSON(): { [k: string]: any };

    /**
     * Gets the default type url for SSL_DetectionRobot
     * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
     * @returns The default type url
     */
    public static getTypeUrl(typeUrlPrefix?: string): string;
}

/** Represents a SSL_DetectionFrame. */
export class SSL_DetectionFrame implements ISSL_DetectionFrame {

    /**
     * Constructs a new SSL_DetectionFrame.
     * @param [properties] Properties to set
     */
    constructor(properties?: ISSL_DetectionFrame);

    /** SSL_DetectionFrame frameNumber. */
    public frameNumber: number;

    /** SSL_DetectionFrame tCapture. */
    public tCapture: number;

    /** SSL_DetectionFrame tSent. */
    public tSent: number;

    /** SSL_DetectionFrame cameraId. */
    public cameraId: number;

    /** SSL_DetectionFrame balls. */
    public balls: ISSL_DetectionBall[];

    /** SSL_DetectionFrame robotsYellow. */
    public robotsYellow: ISSL_DetectionRobot[];

    /** SSL_DetectionFrame robotsBlue. */
    public robotsBlue: ISSL_DetectionRobot[];

    /**
     * Creates a new SSL_DetectionFrame instance using the specified properties.
     * @param [properties] Properties to set
     * @returns SSL_DetectionFrame instance
     */
    public static create(properties?: ISSL_DetectionFrame): SSL_DetectionFrame;

    /**
     * Encodes the specified SSL_DetectionFrame message. Does not implicitly {@link SSL_DetectionFrame.verify|verify} messages.
     * @param message SSL_DetectionFrame message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encode(message: ISSL_DetectionFrame, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Encodes the specified SSL_DetectionFrame message, length delimited. Does not implicitly {@link SSL_DetectionFrame.verify|verify} messages.
     * @param message SSL_DetectionFrame message or plain object to encode
     * @param [writer] Writer to encode to
     * @returns Writer
     */
    public static encodeDelimited(message: ISSL_DetectionFrame, writer?: $protobuf.Writer): $protobuf.Writer;

    /**
     * Decodes a SSL_DetectionFrame message from the specified reader or buffer.
     * @param reader Reader or buffer to decode from
     * @param [length] Message length if known beforehand
     * @returns SSL_DetectionFrame
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decode(reader: ($protobuf.Reader|Uint8Array), length?: number): SSL_DetectionFrame;

    /**
     * Decodes a SSL_DetectionFrame message from the specified reader or buffer, length delimited.
     * @param reader Reader or buffer to decode from
     * @returns SSL_DetectionFrame
     * @throws {Error} If the payload is not a reader or valid buffer
     * @throws {$protobuf.util.ProtocolError} If required fields are missing
     */
    public static decodeDelimited(reader: ($protobuf.Reader|Uint8Array)): SSL_DetectionFrame;

    /**
     * Verifies a SSL_DetectionFrame message.
     * @param message Plain object to verify
     * @returns `null` if valid, otherwise the reason why it is not
     */
    public static verify(message: { [k: string]: any }): (string|null);

    /**
     * Creates a SSL_DetectionFrame message from a plain object. Also converts values to their respective internal types.
     * @param object Plain object
     * @returns SSL_DetectionFrame
     */
    public static fromObject(object: { [k: string]: any }): SSL_DetectionFrame;

    /**
     * Creates a plain object from a SSL_DetectionFrame message. Also converts values to other types if specified.
     * @param message SSL_DetectionFrame
     * @param [options] Conversion options
     * @returns Plain object
     */
    public static toObject(message: SSL_DetectionFrame, options?: $protobuf.IConversionOptions): { [k: string]: any };

    /**
     * Converts this SSL_DetectionFrame to JSON.
     * @returns JSON object
     */
    public toJSON(): { [k: string]: any };

    /**
     * Gets the default type url for SSL_DetectionFrame
     * @param [typeUrlPrefix] your custom typeUrlPrefix(default "type.googleapis.com")
     * @returns The default type url
     */
    public static getTypeUrl(typeUrlPrefix?: string): string;
}
