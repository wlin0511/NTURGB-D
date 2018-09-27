function plot_3DPose(points, nKeyPoints, edges, LiWi, colors )

    scatter3(points(:,1), points(:,3),points(:,2), 400, 'k.', 'LineWidth',1)
    hold on
%     for pointid = 1:nKeyPoints
%         if any(pointid == [5,6,7,13,14,15,16])
%             label_ = [num2str(pointid),'L'];
%         elseif any(pointid == [9,10,11,17,18,19,20])
%             label_ = [num2str(pointid),'R'];
%         else
%             label_ = num2str(pointid);
%         end
%         if ~any(pointid == [8,22,23, 12,24,25])
%             text(points(pointid,1), points(pointid,3),points(pointid,2), label_)
%         end
%     end
    
    for edgeid = 1: size(edges,1)
        plot3(points([edges(edgeid,1), edges(edgeid,2)],1),points([edges(edgeid,1), edges(edgeid,2)],3),...
            points([edges(edgeid,1), edges(edgeid,2)],2), 'LineWidth',LiWi,'Color',colors(edgeid)  )
    end
%     plot3(points([13,14],1), points([13,14],2), points([13,14],3), 'LineWidth',LiWi,'Color','k')
%     plot3(points([13,7],1), points([13,7],2), points([13,7],3), 'LineWidth',LiWi,'Color','k')
%     plot3(points([14,7],1), points([14,7],2), points([14,7],3), 'LineWidth',LiWi,'Color','k')

    
end