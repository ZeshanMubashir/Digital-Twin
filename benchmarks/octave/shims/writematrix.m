function writematrix(A, filename, varargin)
  % Minimal MATLAB writematrix replacement for GNU Octave (CSV output only).
  dlmwrite(filename, A, ',');
end
