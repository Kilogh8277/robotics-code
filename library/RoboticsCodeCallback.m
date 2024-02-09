classdef RoboticsCodeCallback

    methods(Static)


        function urdfpath(callbackContext)
            
            thisURDFParam = get_param(gcb, "urdfpath");
            try
                thisURDF = char(evalin('base', thisURDFParam));
            catch
                if endsWith(thisURDFParam, ".urdf\")
                    thisURDF = strrep(thisURDFParam, ".urdf\", ".urdf");
                else
                    thisURDF = thisURDFParam;
                end
            end
            thisURDF = strrep(thisURDF, "\", "/");
            if startsWith(thisURDF, "./")
                thisURDF = strrep(thisURDF, "./", "");
            end
            set_param(gcb, "urdfpath", strcat("'", thisURDF, "'"));
                try
                    if strcmp(thisURDF, "invalid")
                        return
                    end
                    if exist(thisURDF, "var")
                        thisURDF = eval(thisURDF);
                    elseif ~endsWith(thisURDF, '.urdf')
                        fprintf(2, "ERROR! The file specified is not a URDF: %s\n", thisURDF);
                    end

                    fid = fopen(which(thisURDF), 'r');
                    names = {};
                
                    tline = fgets(fid);
                    i = 1;
                    while (tline ~= -1)
                        if (contains(tline, "<link"))
                            quotes          = strfind(tline, '"');
                            nameIndex       = strfind(tline, 'name=');
                            greaterThanName = quotes(quotes > nameIndex);
                            theseIndices    = (greaterThanName(1)+1):(greaterThanName(2)-1);
                            names{i}        = tline(theseIndices);
                            i = i + 1;
                        end
                        tline = fgets(fid);
                    end
                    
                    thisMask = get_param(gcb, "MaskObject");
                    for i = 1:length(thisMask.Parameters)
                        if strcmp(thisMask.Parameters(i).Name, "source")
                            thisMask.Parameters(i).TypeOptions = names;
                        elseif strcmp(thisMask.Parameters(i).Name, "target")
                            thisMask.Parameters(i).TypeOptions = names;
                        end
                    end
                    
                    fclose(fid);

                catch ME
                    fprintf("%s\n", ME.message)
                    fclose(fid);
                end
        end


        function browsebutton(callbackContext)
            [file, path] = uigetfile("MultiSelect","off", "*.urdf");
            set_param(gcb, "urdfpath", file);
        end


        function source(callbackContext)
            thisSource = get_param(gcb, "source");
            set_param(gcb, "source_string", strcat("'", thisSource, "'"));
        end

        function target(callbackContext)
            thisTarget = get_param(gcb, "target");
            set_param(gcb, "target_string", strcat("'", thisTarget, "'"));
        end
    end
end