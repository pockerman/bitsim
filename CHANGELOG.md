# Changelog

## [4.0.0](https://github.com/pockerman/bitsim/compare/v3.0.0...v4.0.0) (2026-11-20)


### ⚠ BREAKING CHANGES

* rename the library from bitrl to bitsim ([#230](https://github.com/pockerman/bitsim/issues/230))

### Refactoring

* rename the library from bitrl to bitsim ([#230](https://github.com/pockerman/bitsim/issues/230)) ([a58ac27](https://github.com/pockerman/bitsim/commit/a58ac274692752420c38baf2f2204ffa9f35f279))


### Documentation

* limit the build to 4 jobs in the build instructions ([#228](https://github.com/pockerman/bitsim/issues/228)) ([0f4e99d](https://github.com/pockerman/bitsim/commit/0f4e99d5ca2ee08706efe1bf4bd9798c70dc0935))

## [3.0.0](https://github.com/pockerman/bitsim/compare/v2.9.0...v3.0.0) (2026-11-20)


### ⚠ BREAKING CHANGES

* move the reinforcement learning code to bitrl ([#226](https://github.com/pockerman/bitsim/issues/226))

### Refactoring

* move the reinforcement learning code to bitrl ([#226](https://github.com/pockerman/bitsim/issues/226)) ([f7bb58a](https://github.com/pockerman/bitsim/commit/f7bb58a8c898497bfa60e1a9c3a229b35f02f846))

## [2.9.0](https://github.com/pockerman/bitrl/compare/v2.8.0...v2.9.0) (2026-11-20)


### Features

* **dynamics:** fixed wing aircraft with trim, autopilot and a takeof… ([#224](https://github.com/pockerman/bitrl/issues/224)) ([0abdb5d](https://github.com/pockerman/bitrl/commit/0abdb5d88c0dec4d1f5af2d8d8d4a7e247b33b25))


### Refactoring

* remove dead code, rewrite old examples and test what was un… ([#223](https://github.com/pockerman/bitrl/issues/223)) ([96f1b0b](https://github.com/pockerman/bitrl/commit/96f1b0bd5d58cc47f482644e482b203627c07364))


### Documentation

* describe what bitrl does now and show it with gifs ([#220](https://github.com/pockerman/bitrl/issues/220)) ([3f70529](https://github.com/pockerman/bitrl/commit/3f70529583021a3aa024e9d7c45d63c03eed9694))
* fixed wing takeoff and landing gif in the README ([#225](https://github.com/pockerman/bitrl/issues/225)) ([3c19909](https://github.com/pockerman/bitrl/commit/3c19909ceb1f42e21d41defcba62b72e2ea43287))
* inverse kinematics gif ([#222](https://github.com/pockerman/bitrl/issues/222)) ([d6667a5](https://github.com/pockerman/bitrl/commit/d6667a5117ceed630615fa0b9c74547878de9254))

## [2.8.0](https://github.com/pockerman/bitrl/compare/v2.7.0...v2.8.0) (2026-11-19)


### Features

* **estimation:** fuse every sensor of the diff drive robot with an EKF ([#219](https://github.com/pockerman/bitrl/issues/219)) ([098a22b](https://github.com/pockerman/bitrl/commit/098a22b7ea4ff8982986e032e760064fc1bc42a1))
* fly a simulated quadrotor from the laptop with RC commands ([#218](https://github.com/pockerman/bitrl/issues/218)) ([f04f52e](https://github.com/pockerman/bitrl/commit/f04f52ecaaa780ab4adedf5835e959b5e951e8a6))


### Bug Fixes

* **dynamics:** integrate the quadrotor's velocities and attitude corr… ([#216](https://github.com/pockerman/bitrl/issues/216)) ([91442bd](https://github.com/pockerman/bitrl/commit/91442bdcc4d7072a0bc6b7fdfd5a313416dd5c09))

## [2.7.0](https://github.com/pockerman/bitrl/compare/v2.6.0...v2.7.0) (2026-11-19)


### Features

* example 32 ir ultrasound ([#212](https://github.com/pockerman/bitrl/issues/212)) ([4a361ec](https://github.com/pockerman/bitrl/commit/4a361ec360ac1873627eceb8605946612efead39))
* example 34 odometry ([#213](https://github.com/pockerman/bitrl/issues/213)) ([47e5e23](https://github.com/pockerman/bitrl/commit/47e5e237d8a4cd93e9c07823632a8742c3e508e0))
* **examples:** follow a line with infrared sensors and avoid obstacl… ([#210](https://github.com/pockerman/bitrl/issues/210)) ([20c8f92](https://github.com/pockerman/bitrl/commit/20c8f92a8abe754da2f04b008fd8b32dd71a033f))
* **examples:** record example 31 to a video and show a lap on its page ([#209](https://github.com/pockerman/bitrl/issues/209)) ([db009e1](https://github.com/pockerman/bitrl/commit/db009e1db14dbdb6600370ce341947cca31e0973))


### Bug Fixes

* connect2 and diff drive ([#215](https://github.com/pockerman/bitrl/issues/215)) ([b8ae635](https://github.com/pockerman/bitrl/commit/b8ae635b0240e4f70570c52635fec83fe9e74ef0))
* **examples:** say when OpenCV cannot read the stream in example 31 ([#206](https://github.com/pockerman/bitrl/issues/206)) ([7d14315](https://github.com/pockerman/bitrl/commit/7d143156037c0654c9d13a26fcc44f0285fb0c7b))


### Refactoring

* examples progression ([#214](https://github.com/pockerman/bitrl/issues/214)) ([192ae25](https://github.com/pockerman/bitrl/commit/192ae253d041ef9220b213f8ebca151254e33ef2))

## [2.6.0](https://github.com/pockerman/bitrl/compare/v2.5.0...v2.6.0) (2026-11-15)


### Features

* **examples:** drive the square path of the Pi's server in example 31 ([#205](https://github.com/pockerman/bitrl/issues/205)) ([efda063](https://github.com/pockerman/bitrl/commit/efda063bbcce60afc0e2136ca450fed16a73eef3))
* give the diff drive robot a camera and follow a line seen by a … ([#204](https://github.com/pockerman/bitrl/issues/204)) ([5af451b](https://github.com/pockerman/bitrl/commit/5af451b3595909b83843e95cd253ebaa43582ddf))


### Bug Fixes

* **ci:** make the Chrono job pass again ([#201](https://github.com/pockerman/bitrl/issues/201)) ([2915ce4](https://github.com/pockerman/bitrl/commit/2915ce4f46342e31f66c4aa1a62fc4badc5065e1))


### Documentation

* fix the Doxygen warnings in the API documentation ([#203](https://github.com/pockerman/bitrl/issues/203)) ([19e75ab](https://github.com/pockerman/bitrl/commit/19e75ab1e9ed062bdf875bcca6a931642880ad3f))
* keep the examples index and install page in bitrl and check them ([#197](https://github.com/pockerman/bitrl/issues/197)) ([e7df0cc](https://github.com/pockerman/bitrl/commit/e7df0cc60a9d03e516346b1d2381f65a82109838))

## [2.5.0](https://github.com/pockerman/bitrl/compare/v2.4.0...v2.5.0) (2026-11-15)


### Features

* add chrono_world ([d480b8e](https://github.com/pockerman/bitrl/commit/d480b8ea0fe85f4946f06b3be47dc6b23305f993))
* add dc motor class ([0d0a787](https://github.com/pockerman/bitrl/commit/0d0a787cb97b5e02ae6f500f17bab15717a9f505))


### Bug Fixes

* fix bug with quadrotor dynamics matrix ([0ac8231](https://github.com/pockerman/bitrl/commit/0ac8231f9eba60e0d5feb4ee7329b741d9fa54a8))
