import matlab.engine
eng = matlab.engine.start_matlab()
#eng.cd(r'e_posture', nargout=0)
# estimate posture (doPlot, saveVideo, dynamicsOn)
eng.e_posture2(0, 0, 1,nargout=0)

