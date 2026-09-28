function nowSeconds() {
  return Date.now() / 1000.0;
}

function intArrayToJs(values) {
  var out = [];
  for (var i = 0; i < values.length; i++) out.push(values[i]);
  return out;
}

function vecToJs(raw) {
  if (raw === null) return null;
  var M = Java.use('z0.m');
  var v = Java.cast(raw, M);
  return [v.f7144a.value, v.f7145b.value, v.f7146c.value];
}

function poseToJs(raw) {
  if (raw === null) return null;
  return {
    xyz: vecToJs(raw.f5898e.value),
    uvw: vecToJs(raw.f5899f.value)
  };
}

setImmediate(function () {
  Java.perform(function () {
    function emit(payload) {
      console.log('CHICA_FRIDA ' + JSON.stringify(payload));
    }

    function hookServo(className, methodName) {
      try {
        var C = Java.use(className);
        var impl = C[methodName].overload('[I');
        impl.implementation = function (values) {
          emit({
            type: 'servo_call',
            time: nowSeconds(),
            className: className,
            method: methodName,
            values: intArrayToJs(values)
          });
          return impl.call(this, values);
        };
        emit({type: 'hooked', className: className, method: methodName});
      } catch (error) {
        emit({type: 'hook_error', className: className, method: methodName, error: String(error)});
      }
    }

    hookServo('e4.f', 'A');
    hookServo('e4.g', 'B');

    try {
      var Pose = Java.use('p3.a');
      var step = Pose.B.overload('p3.a', 'p3.a', 'double');
      step.implementation = function (target, velocity, dtMs) {
        var result = step.call(this, target, velocity, dtMs);
        emit({
          type: 'set_pose_step',
          time: nowSeconds(),
          dtMs: dtMs
        });
        return result;
      };
      emit({type: 'hooked', className: 'p3.a', method: 'B'});
    } catch (error) {
      emit({type: 'hook_error', className: 'p3.a', method: 'B', error: String(error)});
    }

    try {
      var PoseG = Java.use('p3.a');
      var stepG = PoseG.G.overload('double', 'p3.a', 'boolean', 'double');
      stepG.implementation = function (phase, target, keep, dtMs) {
        var result = stepG.call(this, phase, target, keep, dtMs);
        emit({
          type: 'pose_G_step',
          time: nowSeconds(),
          phaseIn: phase,
          phaseOut: result,
          keep: keep,
          dtMs: dtMs
        });
        return result;
      };
      emit({type: 'hooked', className: 'p3.a', method: 'G'});
    } catch (error) {
      emit({type: 'hook_error', className: 'p3.a', method: 'G', error: String(error)});
    }

    try {
      var Poseg = Java.use('p3.a');
      var stepg = Poseg.g.overload('double', 'boolean', '[Lz0.m;', 'z0.m', 'int', 'int', 'double');
      stepg.implementation = function (phase, allow, anchors, target, gait, style, dtMs) {
        var result = stepg.call(this, phase, allow, anchors, target, gait, style, dtMs);
        emit({
          type: 'pose_g_step',
          time: nowSeconds(),
          phaseIn: phase,
          phaseOut: result,
          allow: allow,
          gait: gait,
          style: style,
          dtMs: dtMs
        });
        return result;
      };
      emit({type: 'hooked', className: 'p3.a', method: 'g'});
    } catch (error) {
      emit({type: 'hook_error', className: 'p3.a', method: 'g', error: String(error)});
    }
  });
});
