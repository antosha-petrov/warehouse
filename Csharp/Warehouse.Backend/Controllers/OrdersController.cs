using Microsoft.AspNetCore.Mvc;
using Warehouse.Backend.Domain;
using Models;


namespace Warehouse.Backend.Controllers
{
    [ApiController]
    [Route("[controller]")]
    public class OrdersController(WarehouseState warehouseState) : ControllerBase
    {
        private readonly WarehouseState _warehouseState = warehouseState;

        [Route("get/app")]
        [HttpGet]
        public ActionResult<Order> GetLastOrderForApp()
        {
            if (_warehouseState.OrderList.Last().Status != "FinishAndLoc")
            {
                return BadRequest(new { message = "Ошибка запроса", code = 400 });
            }

            return _warehouseState.OrderList.Last();
        }

        [Route("get/robot")]
        [HttpGet]
        public ActionResult<Order> GetLastOrderForRobot()
        {
            if (_warehouseState.OrderList.Last().Status != "InProgress")
            {
                return BadRequest(new { message = "Ошибка запроса", code = 400 });
            }

            return _warehouseState.OrderList.Last();
        }

        [Route("get")]
        [HttpGet]
        public ActionResult<Order> GetLastOrder()
        {
            if (_warehouseState.OrderList.Count == 0)
            {
                return Ok("This site is working");
            }

            return _warehouseState.OrderList.Last();
        }

        [Route("create")]
        [HttpPost]
        public ActionResult<Order> CreateOrder()
        {
            var items = _warehouseState.GoodsList
                .Select((goods, index) => new OrderItem(goods, 0))
                .ToList();

            var newOrder = new Order(items);
            _warehouseState.OrderList.Add(newOrder);

            return newOrder;
        }

        [Route("update/loc")]
        [HttpPut]
        public ActionResult<Order> UpdateItemsLocation([FromBody] UpdateLocationRequest request)
        {
            if (request.Locations.Length != 3)
            {
                return BadRequest("Массив должен содержать ровно 3 числа.");
            }

            if (_warehouseState.OrderList.Count == 0)
            {
                return NotFound("Заказов нет.");
            }

            var lastOrder = _warehouseState.OrderList.Last();
            var itemToUpdate = lastOrder.Items.FirstOrDefault(item => item.Goods.Name == request.GoodsName);

            if (itemToUpdate == null)
            {
                return NotFound("Товар с таким именем не найден в последнем заказе.");
            }

            itemToUpdate.Rack = request.Locations[0];
            itemToUpdate.Cell = request.Locations[1];
            itemToUpdate.Shelf = request.Locations[2];


            int j = 0;

            if (j == 0)
            {
                lastOrder.Status = "FinishAndLoc";
            }
            else
            {
                lastOrder.Status = "Finish";
            }

            return lastOrder;
        }


        [Route("update/count")]
        [HttpPut]
        public ActionResult<Order> UpdateLastOrder([FromBody] int[] nums)
        {
            if (nums.Length != 5)
            {
                return BadRequest("Массив должен содержать ровно 5 чисел.");
            }

            if (_warehouseState.OrderList.Count == 0)
            {
                return NotFound("Заказов нет.");
            }

            var lastOrder = _warehouseState.OrderList.Last();
            
            for (var i = 0; i < 5; i++)
            {
                lastOrder.Items[i].Quantity = nums[i];
            }

            return lastOrder;
        }
    }
}
