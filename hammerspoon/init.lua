hs.application.enableSpotlightForNameSearches(true)


-- Reload chrome based on python script. Comes in handy when working with websites..

-- Python script:
-- import os
-- os.system("open -g hammerspoon://someAlert")

hs.urlevent.bind("someAlert", function(eventName, params)
    hs.timer.doAfter(2, reloadchrome)
    
end)


function reloadchrome()
    local app = hs.application.launchOrFocus("Google Chrome")
    hs.eventtap.keyStroke("cmd", "r")
  end


-- Other shit

-- SIDECAR 
