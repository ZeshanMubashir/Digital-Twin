function run_benchmark(script)
  % Run one of the repository's MATLAB simulation scripts headlessly with
  % GNU Octave, so it can be measured by CodSpeed.
  %
  %   script: path to the .m file, relative to the repository root.
  root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
  shims = fullfile(root, 'benchmarks', 'octave', 'shims');

  % Plotting calls are replaced by no-op stubs: only the numerical
  % simulation is measured.
  warning('off', 'Octave:shadowed-function');
  addpath(shims);

  % Fixed seeds make the stochastic (Kalman filter) simulations reproducible.
  randn('state', 42);
  rand('state', 42);

  % Generated CSV files go to a scratch directory, not the repository.
  out = tempname();
  mkdir(out);
  cd(out);

  unwind_protect
    source(fullfile(root, script));
  unwind_protect_cleanup
    % The scripts call `clear`, so the shims path is recomputed here. The real
    % graphics functions are restored because Octave uses them on exit.
    rmpath(fullfile(fileparts(mfilename('fullpath')), 'shims'));
  end_unwind_protect
end
