-- simCodeEditor lua type-checking wrapper
-- (this file is automatically generated: do not edit)
require 'checkargs'

local simCodeEditor=require('simCodeEditor')

__initFunctions=__initFunctions or {}
table.insert(__initFunctions, function()
    local function wrapFunc(funcName,wrapperGenerator)
        _G['simCodeEditor'][funcName]=wrapperGenerator(_G['simCodeEditor'][funcName])
    end

end)

return simCodeEditor