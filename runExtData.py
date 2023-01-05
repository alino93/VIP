import matlab.engine
eng = matlab.engine.start_matlab()
eng.cd('ExtData', nargout=0)
# estimate posture (doPlot, saveVideo, dynamicsOn)
eng.ExtractIMUData(nargout=0)

